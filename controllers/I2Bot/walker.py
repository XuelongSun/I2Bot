import numpy as np
import copy

from scipy.spatial.transform import Rotation as R
import ikpy.chain as ikc

def get_transformation_homo(rot, trans, degrees=True):
    M = np.identity(4)
    M[:3, :3] = R.from_euler("XYZ", rot, degrees=degrees).as_matrix()
    # translation
    M[:3, 3] = trans
    return M

class AntWalker:
    def __init__(self, motors, claws) -> None:
        self.parameters = {}
        self.sequence = {}
        self.sequence_len = 0
        self.initial_pose = {}
        self.current_pose = {}
        self.step_count = 0
        self.motors = motors
        self.claws = claws
        
    def generate_sequence(self):
        raise NotImplementedError
    
    def set_pose(self, pose):
        '''poses is a dict of dicts in format: {"leg_name":[a1,...a5]}
        '''
        for k, v in pose.items():
            for i, a in enumerate(v):
                self.motors['Leg-' + k][i].setPosition(a)
                self.current_pose[k][i] = a

    def set_pose_from_sequence(self):
        pose = {}
        for k, v in self.sequence.items():
            P = [a[self.step_count%self.sequence_len] for a in v]
            pose[k] = P
        self.set_pose(pose)
        self.step_count += 1
        return self.current_pose


class AntWalkerFK(AntWalker):
    def __init__(self, motors, claws) -> None:
        
        super().__init__(motors, claws)
        
        self.parameters = {
            'Gait':'Tripod',
            'HipSwing':np.deg2rad(15),
            'LiftSwing':np.deg2rad(20),
            'StepNum':20,
            'Direction':1, 
            'Rotation':0
        }
        self.sequence = {}
        self.sequence_len = 0
        self.claw_p = -0.2
        self.initial_pose = {
            'FL': [0., 0., -0.32, 1.56, self.claw_p],
            'FR': [0., 0., -0.32, 1.56, self.claw_p],
            'ML': [0., 0, -0.385, 1.388, self.claw_p],
            'MR': [0., 0., -0.385, 1.388, self.claw_p],
            'HL': [0., 0., -0.778, 1.807, self.claw_p],
            'HR': [0., 0., -0.778, 1.807, self.claw_p]
        }
        self.current_pose = copy.deepcopy(self.initial_pose)
        
    def generate_sequence(self):
        # self.parameters
        gait = self.parameters['Gait']
        d_alpha = self.parameters['HipSwing']*self.parameters['Direction']
        d_beta = -self.parameters['LiftSwing']
        d_gamma = self.parameters['LiftSwing']
        step_num = self.parameters['StepNum']
        rotation = self.parameters['Rotation']
        # initial state
        self.sequence = {}
        if gait == 'Tripod':
            for n in filter(lambda x: x.startswith('Leg'), self.motors.keys()):
                k = n.split('-')[1]
                alpha = self.initial_pose[k][0]
                beta = self.initial_pose[k][2]
                gamma = self.initial_pose[k][3]
                beta_s = np.linspace(0, d_beta, int(step_num)) + beta
                beta_s_r = beta_s[::-1]
                beta_s_0 = np.ones(int(len(beta_s)*2))*beta_s[0]
                gamma_s = np.linspace(0, d_gamma, int(step_num)) + gamma
                gamma_s_r = gamma_s[::-1]
                gamma_s_0 = np.ones(int(len(gamma_s)*2))*gamma_s[0]
                if rotation >= 0:
                    if k in ('FL', 'ML', 'HL'):
                        delta_alpha = -d_alpha + rotation*2*d_alpha
                    else:
                        delta_alpha = d_alpha
                else:
                    if k in ('FR', 'MR', 'HR'):
                        delta_alpha = d_alpha + rotation*2*d_alpha
                    else:
                        delta_alpha = -d_alpha
                alpha_s = np.linspace(-delta_alpha, delta_alpha, int(step_num*2)) + alpha
                alpha_s_r = alpha_s[::-1]
                alpha_seq_a = np.hstack([alpha_s, alpha_s_r])
                alpha_seq_b = np.hstack([alpha_s_r, alpha_s])
                if k in ('MR', 'FL', 'HL'):
                    self.sequence[k] = [alpha_seq_a,
                                        np.ones(int(step_num*4))*self.initial_pose[k][1],
                                        np.hstack([beta_s, beta_s_r, beta_s_0]),
                                        np.hstack([gamma_s, gamma_s_r, gamma_s_0]),
                                        np.ones(int(step_num*4))*self.initial_pose[k][4],
                                        ]
                else:
                    self.sequence[k] = [alpha_seq_b,
                                        np.ones(int(step_num*4))*self.initial_pose[k][1],
                                        np.hstack([beta_s_0, beta_s, beta_s_r]),
                                        np.hstack([gamma_s_0, gamma_s, gamma_s_r]),
                                        np.ones(int(step_num*4))*self.initial_pose[k][4],
                                        ]
        elif gait == 'Ripple':
            moduloMap = {'HL':0, 'FR':1, 'ML':2, 'HR':3, 'FL':4, 'MR':5}
            for n in filter(lambda x: x.startswith('Leg'), self.motors.keys()):
                k = n.split('-')[1]
                alpha = self.initial_pose[k][0]
                beta = self.initial_pose[k][2]
                gamma = self.initial_pose[k][3]                
                beta_lift = np.linspace(0, d_beta, step_num) + beta
                gamma_lift = np.linspace(0, d_gamma, step_num) + gamma
                if rotation != 0:
                    delta_alpha = d_alpha
                else:
                    if k in ('FL', 'ML', 'HL'):
                        delta_alpha = -d_alpha
                    else:
                        delta_alpha = d_alpha
                
                fw1 = np.linspace(-delta_alpha, 0, step_num) + alpha
                fw2 = np.linspace(0, delta_alpha, step_num) + alpha
                half_d = delta_alpha/2
                bk1 = np.linspace(delta_alpha, half_d, step_num) + alpha
                bk2 = np.linspace(half_d, 0, step_num) + alpha
                bk3 = np.linspace(0, -half_d, step_num) + alpha
                bk4 = np.linspace(-half_d, -delta_alpha, step_num) + alpha
                
                bn = np.ones(int(step_num*4))*beta
                gn = np.ones(int(step_num*4))*gamma
                alpha_s = np.hstack([fw1, fw2, bk1, bk2, bk3, bk4])
                beta_s = np.hstack([beta_lift, beta_lift[::-1], bn])
                gamma_s = np.hstack([gamma_lift, gamma_lift[::-1], gn])
                self.sequence[k] = [np.roll(alpha_s, int(-moduloMap[k]*step_num)),
                                    np.ones(int(step_num*6))*self.initial_pose[k][1],
                                    np.roll(beta_s, int(-moduloMap[k]*step_num)),
                                    np.roll(gamma_s, int(-moduloMap[k]*step_num)),
                                    np.ones(int(step_num*6))*self.initial_pose[k][4]
                                    ]
        elif gait == 'Wave':
            moduloMap = {'FL':0, 'ML':5, 'HL':4, 'FR':3, 'MR':2, 'HR':1}
            for n in filter(lambda x: x.startswith('Leg'), self.motors.keys()):
                k = n.split('-')[1]
                alpha = self.initial_pose[k][0]
                beta = self.initial_pose[k][2]
                gamma = self.initial_pose[k][3]                
                if rotation != 0:
                    delta_alpha = d_alpha
                else:
                    if k in ('FL', 'ML', 'HL'):
                        delta_alpha = -d_alpha
                    else:
                        delta_alpha = d_alpha
                fw1 = np.linspace(-delta_alpha, 0, step_num) + alpha
                fw2 = np.linspace(0, delta_alpha, step_num) + alpha
                alpha_unit = delta_alpha/5
                bks = []
                for i in range(10):
                    t_ = np.linspace(delta_alpha - i*alpha_unit, 
                                     delta_alpha - (i+1)*alpha_unit, step_num) + alpha
                    bks.append(t_)

                bn = np.ones(int(step_num*10))*beta
                gn = np.ones(int(step_num*10))*gamma
                beta_lift = np.linspace(0, d_beta, step_num) + beta
                gamma_lift = np.linspace(0, d_gamma, step_num) + gamma
                
                alpha_s = np.hstack([fw1, fw2] + bks)
                beta_s = np.hstack([beta_lift, beta_lift[::-1], bn])
                gamma_s = np.hstack([gamma_lift, gamma_lift[::-1], gn])
                self.sequence[k] = [np.roll(alpha_s, int(-moduloMap[k]*step_num*2)),
                                    np.ones(int(step_num*12))*self.initial_pose[k][1],
                                    np.roll(beta_s, int(-moduloMap[k]*step_num*2)),
                                    np.roll(gamma_s, int(-moduloMap[k]*step_num*2)),
                                    np.ones(int(step_num*12))*self.initial_pose[k][4]
                                    ]
        else:
            pass
        
        self.sequence_len = len(self.sequence['MR'][0])


class AntWalkerIK(AntWalker):
    def __init__(self, motors, claws) -> None:
        
        super().__init__(motors, claws)
        
        self.parameters = {
            'Gait':'Tripod',
            'Length': 0.02,
            'Height': 0.02,
            'StepNum':60,
            'Rotation':0
            }
        
        self.sequence = {}
        self.sequence_len = 0

        
        # save initial body contact points
        self.body_contact_points = {
            'FL':np.array([0.24-0.35, -0.08, -0.02, 1]),
            'FR':np.array([0.24-0.35, 0.08, -0.02, 1]),
            'ML':np.array([0.35-0.35, -0.05, -0.03, 1]),
            'MR':np.array([0.35-0.35, 0.05, -0.03, 1]),
            'HL':np.array([0.43-0.35, -0.07, -0.03, 1]),
            'HR':np.array([0.43-0.35, 0.07, -0.03, 1]),
        }
        # save initial rotation
        self.leg_init_rotation = {
            'FL':3*np.pi/4,
            'FR':np.pi/4,
            'ML':np.pi,
            'MR':0,
            'HL':-3*np.pi/4,
            'HR':-np.pi/4,
        }
        # leg chain
        self.leg_chains = {
            'F': ikc.Chain.from_urdf_file("urdf/front_leg.urdf",
                                          active_links_mask=[False, True, True, True, True, False]), 
            'M': ikc.Chain.from_urdf_file("urdf/middle_leg.urdf",
                                          active_links_mask=[False, True, True, True, True, False]), 
            'H': ikc.Chain.from_urdf_file("urdf/hind_leg.urdf",
                                          active_links_mask=[False, True, True, True, True, False])
        }
        self.claw_p = -0.2
        self.initial_pose = {
            'FL': [0., 0., -0.32, 1.56, self.claw_p],
            'FR': [0., 0., -0.32, 1.56, self.claw_p],
            'ML': [0., 0, -0.385, 1.388, self.claw_p],
            'MR': [0., 0., -0.385, 1.388, self.claw_p],
            'HL': [0., 0., -0.778, 1.807, self.claw_p],
            'HR': [0., 0., -0.778, 1.807, self.claw_p]
        }
        # self.initial_pose = {
        #     'FL': [0., 0., -0.32, 1.56, 0.],
        #     'FR': [0., 0., -0.32, 1.56, 0.],
        #     'ML': [0., 0, -0.385, 1.388, 0.],
        #     'MR': [0., 0., -0.385, 1.388, 0.],
        #     'HL': [0., 0., -0.778, 1.807, 0.],
        #     'HR': [0., 0., -0.778, 1.807, 0.]
        # }
        self.current_pose = copy.deepcopy(self.initial_pose)
    
    def solve_leg_ik(self, leg, target_position):
        ik = self.leg_chains[leg[0]].inverse_kinematics(target_position=np.array(target_position)*100)
        pose = {leg: [ik[1], 0, ik[2], ik[3], ik[4]]}
        return pose
    
    def solve_body_ik(self, rot, trans):
        pose = {}
        # get homo transformation matrix
        tm = get_transformation_homo(rot, trans)
        for k, v in self.body_contact_points.items():
            p = tm.dot(v)
            offset = v - p
            
            initial_angle = [
                0,
                self.initial_pose[k][0],
                self.initial_pose[k][2],
                self.initial_pose[k][3],
                self.initial_pose[k][4],
                0,
            ]
            
            p_old = self.leg_chains[k[0]].forward_kinematics(initial_angle)[:, 3]
            
            a_ = self.leg_init_rotation[k]
            
            target_position = [
                offset[0]*np.cos(a_)*100 + offset[1]*np.sin(a_)*100 + p_old[0],
                -offset[0]*np.sin(a_)*100 + offset[1]*np.cos(a_)*100 + p_old[1],
                offset[2]*100 + p_old[2]
            ]
            
            ik = self.leg_chains[k[0]].inverse_kinematics(target_position=target_position,
                                                          initial_position=initial_angle)
            
            pose[k] = [ik[1], 0, ik[2], ik[3], ik[4]]
        
        return pose
    
    def generate_sequence(self):
        # read parameters of gait
        gait = self.parameters['Gait']
        length = self.parameters['Length']
        height = self.parameters['Height']
        step_num = self.parameters['StepNum']
        rotate = self.parameters['Rotation']
        stance_pose = []
        swing_pose = []
        self.sequence = {}
        if gait == 'Tripod':
            stance_y = np.linspace(length, -length, int(step_num), endpoint=True)
            swing_y = np.linspace(-length, length, int(step_num), endpoint=True)
            swing_z = np.linspace(0, height, int(step_num), endpoint=True)
            rotate_z = np.linspace(0, rotate, int(step_num), endpoint=True)
            rotate_z_r = np.linspace(rotate, 0, int(step_num), endpoint=True)
            for r1, r2, sty, swy, swz in zip(rotate_z, rotate_z_r, stance_y, swing_y, swing_z):
                stance_pose.append(self.solve_body_ik([0, 0, r1],[0, -sty, 0]))
                swing_pose.append(self.solve_body_ik([0, 0, r2],[0, -swy, -swz]))
            
            for n in filter(lambda x: x.startswith('Leg'), self.motors.keys()):
                leg = n.split('-')[1]
                if leg in ('MR', 'FL', 'HL'):
                    self.sequence[leg] = []
                    for i in range(5):
                        self.sequence[leg].append([p[leg][i] for p in stance_pose] + [p[leg][i] for p in swing_pose])
                
                else:
                    self.sequence[leg] = []
                    for i in range(5):
                        self.sequence[leg].append([p[leg][i] for p in swing_pose] + [p[leg][i] for p in stance_pose])
        
        self.sequence_len = len(self.sequence['MR'][0])