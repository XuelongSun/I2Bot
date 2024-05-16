from controller import Robot, Supervisor
from walker import AntWalkerFK, AntWalkerIK
from utils import Logger, pi2pi, ZernikeMoment
from navigation import *

import cv2

class AntBase(Supervisor):
    def __init__(self, name='test'):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())

        self.logger = Logger('data/', name)
        self.logger.log(self.__class__.__name__ + ' instance created.')
        
        # timer
        self.loop_count = 0
        self.timer = self.getTime()
        
        # keyboard process
        self.keyborad = self.getKeyboard()
        self.keyborad.enable(self.time_step)
        self.last_key_time = self.getTime()
        self.key_interval = 0.1
        
        # cameras - eyes
        self.left_camera = self.getDevice("left_camera")
        self.left_camera.enable(self.time_step)
        self.lr_img_size = (self.left_camera.getWidth(), self.left_camera.getHeight())
        self.right_camera = self.getDevice("right_camera")
        self.right_camera.enable(self.time_step)
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.img_size = (self.camera.getWidth(), self.camera.getHeight())
        
        # olfactory sensors
        self.odor_trail_sensor_left = self.getDevice("DSensorL")
        self.odor_trail_sensor_left.enable(self.time_step)
        self.odor_trail_sensor_right = self.getDevice("DSensorR")
        self.odor_trail_sensor_right.enable(self.time_step)
        
        self.odor_plume_sensor_left = self.getDevice("OSensorL")
        self.odor_plume_sensor_left.enable(self.time_step)
        self.odor_plume_sensor_right = self.getDevice("OSensorR")
        self.odor_plume_sensor_right.enable(self.time_step)
        
        # pen for drawing trajectory
        self.pen = self.getDevice("pen")
        self.pen.setInkColor(0xF09B18, 1.0)
        
        # motors
        self.motors = {}
        self.p_sensors = {}
        self.t_sensor = {}
        ## legs
        for p1 in ['F', 'M', 'H']:
            for p2 in ['L', 'R']:
                temp_p = []
                temp_m = []
                for i in range(1, 6):
                    temp_p.append(self.getDevice('PSensorLeg' + p2 + p1 + str(i)))
                    temp_p[-1].enable(self.time_step)
                    temp_m.append(self.getDevice('MotorLeg' + p2 + p1 + str(i)))
                    temp_m[-1].enableTorqueFeedback(self.time_step)
                self.p_sensors['Leg-'+ p1 + p2] = temp_p.copy()
                self.motors['Leg-' + p1 + p2] = temp_m.copy()
                
                self.t_sensor['Leg-' + p1 + p2] = self.getDevice('TSensorLeg' + p2 + p1)
                self.t_sensor['Leg-' + p1 + p2].enable(self.time_step)
        ## antenna
        for d in ['L', 'R']:
            _m = []
            _p = []
            for i in range(1,4):
                _m.append(self.getDevice('MotorAntennae' + d + str(i)))
                _p.append(self.getDevice('PSensorAntennae' + d + str(i)))
                _p[-1].enable(self.time_step)
            self.motors['Antennae-' + d] = _m.copy()
            self.p_sensors['Antennae-' + d] = _p.copy()
        ## head and abdomen
        for n in ['Head', 'Abdomen']:
            _m = []
            _p = []
            for d in ['Y', 'Z']:
                _m.append(self.getDevice('Motor' + n + d))
                _p.append(self.getDevice('PSensor' + n + d))
                _p[-1].enable(self.time_step)
            self.motors[n] = _m.copy()
            self.p_sensors[n] = _p.copy()
        
        # the tip of each legs
        self.claws = {}
        for n in ['FL', 'ML', 'HL', 'FR', 'MR', 'HR']:
            self.claws['Leg-' + n] = self.getFromDef(n + '_TarsusClaw')
        self.enable_adhesion = False
        self.adhesion_force = 120

    def set_pose(self, pose):
        for k, v in pose.items():
            for i, a in enumerate(v):
                self.motors['Leg-' + k][i].setPosition(a)
        
    def my_step(self, func=None, fargs=[]):
        '''step the simulation, call func if it is not None'''
        if self.step(self.time_step) == -1:
            return 1
        else:
            self.loop_count += 1
            self.timer = self.getTime()
            if self.enable_adhesion:
                for k, v in self.claws.items():
                    if self.t_sensor[k].getValue() > 0.1:
                        v.addForce([self.adhesion_force, 0, 0], False)
                    
            if func is not None:
                func(*fargs)
            return 0

    def wait(self, ms):
        '''wait for ms milliseconds'''
        start_time = self.getTime()
        s = ms / 1000.0
        while s + start_time >= self.getTime():
            self.my_step()

    def get_self_position(self):
        sf = self.getSelf().getField('translation')
        return sf.getSFVec3f()

    def set_self_position(self, x, y):
        sf = self.getSelf().getField('translation')
        sf.setSFVec3f([x, y, sf.getSFVec3f()[-1]])

    def get_self_rotation(self):
        sf = self.getSelf().getField('rotation')
        return sf.getSFRotation()

    def set_self_rotation(self, heading):
        sf = self.getSelf().getField('rotation')
        sf.setSFRotation([0.0, 0.0, 1.0, heading])
    
    def get_self_velocity(self):
        sf = self.getSelf().getVelocity()
        return sf[:2]
    
    def get_self_angular_velocity(self):
        sf = self.getSelf().getVelocity()
        return sf[3:]
    
    def get_leg_motors_position(self):
        p = {}
        for k, v in self.p_sensors.items():
            p[k] = []
            for i in v:
                p[k].append(i.getValue())
        return p
    
    def _get_img(self, camera, size, gray=True):
        img = np.fromstring(camera.getImage(), np.uint8).reshape(size[1], size[0], 4)
        if gray:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        return img
    
    def get_binocular_img(self, gray=True):
        img_l = self._get_img(self.left_camera, self.lr_img_size, gray)
        img_r = self._get_img(self.right_camera, self.lr_img_size, gray)
        return img_l, img_r
    
    def get_panoramic_img(self, gray=True):
        img = self._get_img(self.camera, self.img_size, gray)
        return img
    
    def keyboard_process(self):
        t = self.getTime()
        key = self.keyborad.getKey()
        if key == ord('F') and t - self.last_key_time > self.key_interval:
            self.last_key_time = self.getTime()


class GaitController(AntBase):
    def __init__(self, name='test'):
        super().__init__(name)
        self.walker_fk = AntWalkerFK(self.motors, self.claws)
        self.walker_ik = AntWalkerIK(self.motors, self.claws)
        self.walker = None
        
        # variables for logging
        for ps in self.motors.keys():
            if ps.startswith('Leg'):
                self.logger.data['ps-' + ps] = []
                self.logger.data['ts-' + ps] = []
                self.logger.data['torque-' + ps] = []
        self.logger.data['time'] = []
        self.logger.data['velocity'] = []
        self.logger.data['position'] = []
    
    def log(self):
        for k, v in self.p_sensors.items():
            if k.startswith('Leg'):
                self.logger.data['ps-' + k].append([i.getValue() for i in v])
                self.logger.data['torque-' + k].append([i.getTorqueFeedback() for i in self.motors[k]])
        for k, v in self.t_sensor.items():
            if k.startswith('Leg'):
                self.logger.data['ts-' + k].append(v.getValue())
        self.logger.data['time'].append(self.getTime())
        self.logger.data['velocity'].append(self.getSelf().getVelocity())
        self.logger.data['position'].append(self.get_self_position())
        
    def set_walker_type(self, mode):
        if mode == 'IK':
            self.walker_ik.generate_sequence()
            self.walker = self.walker_ik
            self.logger.log(f'Sequence (L={self.walker.sequence_len}) for IK walker generated.')
        elif mode == 'FK':
            self.walker_fk.generate_sequence()
            self.walker = self.walker_fk
            self.logger.log(f'Sequence (L={self.walker.sequence_len}) for FK walker generated.')
        
        print("walk type is set to: ", mode)
        self.logger.log('walk type is set to: ' + mode)
        return self.walker
        
    def walk(self, num_loop=None, record_para=None):
        self.logger.log('Start walking...')
        if self.walker is None:
            self.walker = self.walker_fk
            print("Warning: walker is not set, use FK by default.")
            self.logger.log('Warning--walker is not set, use FK by default.')
        # reset the step count
        self.walker.step_count = 0
        
        if record_para['video']:
            self.movieStartRecording(record_para['record_file_path'] + self.logger.name + '_video.mp4',
                                     1280, 720,
                                     0, 40, 1, False)
            self.logger.log('Start video recording...')
        if num_loop is None:
            # walk continuously
            while not self.my_step(self.log):
                self.walker.set_pose_from_sequence()
        else:
            # walk for num_loop of cycles
            for _ in range(num_loop*self.walker.sequence_len):
                self.walker.set_pose_from_sequence()
                self.my_step(self.log)
                if self.loop_count in record_para['snapshots']:
                    self.exportImage(record_para['record_file_path'] + self.logger.name + '_snapshots_' + str(self.loop_count) + '.jpg', 90)
                    self.logger.log('Snapshot at loop: ' + str(self.loop_count))

        self.logger.log('Walking finished.')
        if record_para['video']:
            self.movieStopRecording()
            self.logger.log('Video recording finished.')
            while not self.movieIsReady():
                self.my_step()
            self.logger.log('Video file saved.')


class NavigationController(AntBase):
    def __init__(self, name='test'):
        super().__init__(name)
        
        self.walker = AntWalkerFK(self.motors, self.claws)
        self.gait_ctl_loop_num = 1
        
        self.cx = CentralComplexModel()
        self.cx.noise = 0.0008
        self.pi = PathIntegration()
        self.vb = VisualBeacon()
        
        self.motor_k = 2.0

        # pen for drawing trajectory
        self.pen = self.getDevice("pen")
        self.pen.setInkColor(0x548235, 1.0)

        # states
        self.current_position = self.get_self_position()
        self.previous_position = np.array(self.current_position).copy()
        self.heading = self.get_self_rotation()[-1]
        self.velocity = self.velocity = np.array([0, 0])
        print('Initial heading:', self.heading, 'Initial velocity:', self.velocity)
        self.logger.log('Initial heading: ' + str(self.heading) + 'Initial velocity: ' + str(self.velocity))
        
        # variables for logging
        for ps in self.motors.keys():
            if ps.startswith('Leg'):
                self.logger.data['ps-' + ps] = []
                self.logger.data['ts-' + ps] = []
                self.logger.data['torque-' + ps] = []
        self.logger.data['time'] = []
        self.logger.data['velocity'] = []
        self.logger.data['position'] = []
        
        # PI
        self.logger.data['CPU4'] = []
        self.logger.data['motor'] = []
        self.logger.data['heading'] = []
        
        # visual beacons
        self.logger.data['vb_desired'] = []
        # visual compass
        self.logger.data['vs_node_dir'] = []
        self.logger.data['zm_phase'] = []
        self.logger.data['angular_velocity'] = []
        
        # odor trail following
        self.logger.data['odor_l'] = []
        self.logger.data['odor_r'] = []
        # odor plume tracking
        self.logger.data['wind_l'] = []
        self.logger.data['wind_r'] = []

    def log(self):
        for k, v in self.p_sensors.items():
            if k.startswith('Leg'):
                self.logger.data['ps-' + k].append([i.getValue() for i in v])
                self.logger.data['torque-' + k].append([i.getTorqueFeedback() for i in self.motors[k]])
        for k, v in self.t_sensor.items():
            if k.startswith('Leg'):
                self.logger.data['ts-' + k].append(v.getValue())
        self.logger.data['time'].append(self.getTime())
        self.logger.data['velocity'].append(self.get_self_velocity())
        self.logger.data['position'].append(self.get_self_position())

        self.logger.data['CPU4'].append(self.pi.memory.tolist())
        self.logger.data['motor'].append(self.motor_f)
    
    def vb_log(self):
        self.logger.data['time'].append(self.getTime())
        self.logger.data['velocity'].append(self.get_self_velocity())
        self.logger.data['position'].append(self.get_self_position())
        self.logger.data['heading'].append(self.get_self_rotation()[-1] + np.pi)
        self.logger.data['vb_desired'].append(self.vb.output.tolist())

    def pi_foraging(self, start_position=None, start_heading=None, timeout=100, show_trajectory=True, trajectory_color=0x2F5597, trajectory_density=0.2):
        interval_step = interval_step = self.prepare_navigation(start_position, start_heading, show_trajectory, trajectory_color, trajectory_density)
        
        for _ in range(int(timeout*1000/self.time_step)):
            if self.walker.step_count % interval_step == 0:
                self.current_position = self.get_self_position()
                self.heading = self.get_self_rotation()[-1]
                self.velocity = np.array([(self.current_position[0] - self.previous_position[0])/(self.time_step*interval_step)*1000,
                                          (self.current_position[1] - self.previous_position[1])/(self.time_step*interval_step)*1000])
                self.previous_position = np.array(self.current_position).copy()
                self.cx.global_current_heading(pi2pi(self.heading - np.pi))
                self.pi.update_neuron_activation(self.cx, pi2pi(self.heading - np.pi), self.velocity)
                self.motor_f = np.random.uniform() * 0.26 - 0.1
                self.walker.parameters['Rotation'] = self.motor_f
                self.walker.parameters['HipSwing'] = (1.0 - np.min([abs(self.motor_f), 1.0])) * np.deg2rad(20) + np.deg2rad(5)
                self.walker.generate_sequence()
            
            self.walker.set_pose_from_sequence()
            self.my_step(self.log)
    
    def prepare_navigation(self, start_position=None, start_heading=None,
               show_trajectory=True, trajectory_color=0x548235, trajectory_density=0.2):
        self.pen.write(show_trajectory)
        self.pen.setInkColor(trajectory_color, trajectory_density)
        
        if self.walker.sequence_len == 0:
            self.walker.generate_sequence()
        
        interval_step = self.walker.sequence_len*self.gait_ctl_loop_num
        
        if start_position:
            self.set_self_position(start_position[0], start_position[1])
        if start_heading:
            self.set_self_rotation(start_heading)
        
        return interval_step
    
    
    def pi_homing(self, start_position=None, start_heading=None,
               show_trajectory=True, trajectory_color=0x548235, trajectory_density=0.2,
               timeout=10):
        
        interval_step = self.prepare_navigation(start_position, start_heading, show_trajectory, trajectory_color, trajectory_density)
        
        print('Start homing PI at t={}(c={})...'.format(self.timer, self.loop_count))
        self.logger.log('Start homing PI at t={}(c={})...'.format(self.timer, self.loop_count))
        for _ in range(int(timeout*1000/self.time_step)):
            if self.loop_count % interval_step == 0:
                self.current_position = self.get_self_position()
                self.heading = self.get_self_rotation()[-1]
                self.velocity = np.array([(self.current_position[0] - self.previous_position[0])/(self.time_step*interval_step)*1000,
                                          (self.current_position[1] - self.previous_position[1])/(self.time_step*interval_step)*1000])
                self.previous_position = np.array(self.current_position).copy()

                self.cx.global_current_heading(pi2pi(self.heading - np.pi))
                self.pi.update_neuron_activation(self.cx, pi2pi(self.heading - np.pi), self.velocity)
                self.cx.current_heading_memory = self.cx.I_tb1
                self.cx.desired_heading_memory = self.pi.memory
                    
                self.cx.steering_circuit_out()
                self.motor_f = self.cx.motor_value*self.motor_k
                self.walker.parameters['Rotation'] = self.motor_f
                self.walker.parameters['HipSwing'] = (1.0 - np.min([abs(self.motor_f), 1.0])) * np.deg2rad(20) + np.deg2rad(5)
                self.walker.generate_sequence()
            
            self.walker.set_pose_from_sequence()
            self.my_step(self.log)
            
            if np.sqrt(np.sum(np.square(self.current_position))) < 0.8:
                print('Homing finished at t={}(c={})...'.format(self.timer, self.loop_count))
                self.logger.log('Homing finished at t={}(c={})...'.format(self.timer, self.loop_count))
                break
        if _ > int(timeout*1000/self.time_step):
            print('Homing timeout at t={}(c={})...'.format(self.timer, self.loop_count))
            self.logger.log('Homing timeout at t={}(c={})...'.format(self.timer, self.loop_count))
    
    def walk(self, num_loop=None):
        # reset the step count
        self.walker.step_count = 0
        if num_loop is None:
            # walk continuously
            while not self.my_step():
                self.walker.set_pose_from_sequence()
        else:
            # walk for num_loop of cycles
            for _ in range(num_loop*self.walker.sequence_len):
                self.walker.set_pose_from_sequence()
                self.my_step()
                
    def visual_beacons(self, start_position=None, start_heading=None, mode='right', landmark_p=np.array([0, -25]), landmark_r=1,
                       show_trajectory=True, trajectory_color=0x548235, trajectory_density=0.2,
                       timeout=10):
        interval_step = self.prepare_navigation(start_position, start_heading, show_trajectory, trajectory_color, trajectory_density)
        print('Start visual beacons at t={}(c={})...'.format(self.timer, self.loop_count))
        self.logger.log('Start visual beacons at t={}(c={})...'.format(self.timer, self.loop_count))
        for _ in range(int(timeout*1000/self.time_step)):
            if self.loop_count % interval_step == 0:
                img_l, img_r = self.get_binocular_img(gray=True)
                self.cx.global_current_heading(self.heading + np.pi)
                self.vb.update_neuron_activation(img_l, img_r, self.cx, mode=mode)
                self.cx.current_heading_memory = self.cx.I_tb1
                self.cx.desired_heading_memory = np.hstack([self.vb.output, self.vb.output])
            
                self.cx.steering_circuit_out()
                self.motor_f = self.cx.motor_value*self.motor_k
                self.walker.parameters['Rotation'] = self.motor_f
                self.walker.parameters['HipSwing'] = (1.0 - np.min([abs(self.motor_f), 1.0])) * np.deg2rad(20) + np.deg2rad(5)
                self.walker.generate_sequence()
            
            self.walker.set_pose_from_sequence()
            self.my_step(self.vb_log)
            self.current_position = np.array(self.get_self_position())
            if np.sqrt(np.sum(np.square(self.current_position[:2]-landmark_p))) < landmark_r*2:
                print('Visual beacons finished at t={}(c={})...'.format(self.timer, self.loop_count))
                self.logger.log('Visual beacons finished at t={}(c={})...'.format(self.timer, self.loop_count))
                break
        if _ > int(timeout*1000/self.time_step):
            print('Visual beacons timeout at t={}(c={})...'.format(self.timer, self.loop_count))
            self.logger.log('Visual beacons timeout at t={}(c={})...'.format(self.timer, self.loop_count))
    
    
    def visual_compass(self, start_position=None, start_heading=None, vs_mode='incremental',
                       show_trajectory=True, trajectory_color=0x548235, trajectory_density=0.2,
                       timeout=10):
        interval_step = self.prepare_navigation(start_position, start_heading, show_trajectory, trajectory_color, trajectory_density)
        # create ZM calculate
        zm = ZernikeMoment(size=int(self.img_size[0]))
        print('Create Zernike Moment object with size:', zm.size)
        # get the visual scene
        vs_node = self.getFromDef('VisualScene')
        vs_node_dir = 0
        self.walker.parameters['HipSwing'] = 0
        self.walker.generate_sequence()
        # run a bit
        for _ in range(self.walker.sequence_len):
            self.walker.set_pose_from_sequence()
            self.my_step()
        
        img = self.get_panoramic_img()
        _, _, init_phase = zm.compute(img, 7, 1)
        # _, _, init_phase = zernike_moment(img, 7, 1)
        print('Start visual compass at t={}(c={})...'.format(self.timer, self.loop_count))
        self.logger.log('Start visual compass at t={}(c={})...'.format(self.timer, self.loop_count))
        for _ in range(int(timeout*1000/(self.time_step*interval_step))):
            if vs_mode == 'incremental':
                vs_node.getField('rotation').setSFRotation([0, 0, 1, pi2pi(vs_node_dir)])
                vs_node_dir += 0.01*np.random.randint(-1, 3)
            elif vs_mode == 'jump':
                if self.loop_count % 600*interval_step == 0:
                    # generate a random angle between -np.pi/2 and np.pi/2
                    
                    vs_node_dir = np.random.uniform(-np.pi, np.pi)
                    vs_node.getField('rotation').setSFRotation([0, 0, 1, pi2pi(vs_node_dir)])
                    
            img = self.get_panoramic_img()
            _, _, phase = zm.compute(img, 7, 1)
            # _, _, phase = zernike_moment(img, 7, 1)
            # print('p0:', init_phase, ',p1:', phase)
            if abs(phase - init_phase) > 0.8:
                self.walker.parameters['Rotation'] = np.sign(phase - init_phase)
                self.walker.parameters['HipSwing'] = np.deg2rad(abs(phase - init_phase)/5)
                self.walker.generate_sequence()
            
            for _ in range(self.walker.sequence_len):
                self.walker.set_pose_from_sequence()
                self.logger.data['zm_phase'].append(phase)
                self.logger.data['vs_node_dir'].append(vs_node_dir)
                self.logger.data['time'].append(self.getTime())
                self.logger.data['angular_velocity'].append(self.get_self_angular_velocity())
                self.logger.data['heading'].append(self.get_self_rotation()[-1])
                
                self.my_step()
            # self.walk(1)
        
        print('Visual compass ended at t={}(c={})...'.format(self.timer, self.loop_count))
        self.logger.log('Visual compass ended at t={}(c={})...'.format(self.timer, self.loop_count))
    
    def get_trail_odor(self):
        o_l = self.odor_trail_sensor_left.getValue()
        o_r = self.odor_trail_sensor_right.getValue()
        return o_l, o_r
    
    def odor_trial_following(self, start_position=None, start_heading=None, timeout=10, antenna_mode='move'):
        initial_al_p = 0.7
        if antenna_mode == 'move':
            move_amp = 0.2
        elif antenna_mode == 'fix':
            move_amp = 0
        else:
            raise ValueError('Antenna mode not supported.')
        self.motors['Antennae-L'][0].setPosition(-0.44)
        self.motors['Antennae-R'][0].setPosition(-0.44)
        
        self.motors['Antennae-L'][1].setPosition(initial_al_p)
        self.motors['Antennae-R'][1].setPosition(-initial_al_p)
        print('Start odor trail following at t={}(c={})...'.format(self.timer, self.loop_count))
        self.logger.log('Start odor trail following at t={}(c={})...'.format(self.timer, self.loop_count))
        for _ in range(int(timeout*1000/self.time_step)):
            o_l, o_r = self.get_trail_odor()
            no_odor = (o_l < 900) and (o_r < 900)
            self.motors['Antennae-L'][1].setPosition(initial_al_p + np.random.rand()*move_amp)
            self.motors['Antennae-R'][1].setPosition(-initial_al_p + np.random.rand()*move_amp)
            if not no_odor:
                d = o_l - o_r
                if abs(d) > 5:
                    # hs = 6 + np.random.randint(0, 3)
                    hs = 4
                    r = d/400
                    n = 1
                else:
                    hs = 8
                    r = 0
                    n = 1
            else:
                hs = 3
                r = np.random.choice([-1, 1])
                n = 1
            # print(o_l, o_r, no_odor, hs, r, n)
            # self.walker.parameters['HipSwing'] = np.deg2rad(hs + np.random.randint(-1, 2))
            self.walker.parameters['HipSwing'] = np.deg2rad(hs)
            self.walker.parameters['Rotation'] = np.sign(r)
            self.walker.generate_sequence()
            # self.walk(n)
            for _ in range(self.walker.sequence_len*n):
                self.walker.set_pose_from_sequence()
                self.my_step()
                self.logger.data['time'].append(self.getTime())
                self.logger.data['odor_l'].append(o_l)
                self.logger.data['odor_r'].append(o_r)
                self.logger.data['position'].append(self.get_self_position())
                self.logger.data['heading'].append(self.get_self_rotation()[-1])
                self.logger.data['velocity'].append(self.get_self_velocity())
                self.logger.data['angular_velocity'].append(self.get_self_angular_velocity())
                
            if self.get_self_position()[0] < -5.5:
                self.logger.log('Odor trail following finished at t={}.'.format(self.timer))
                print('Odor trail following finished at t={}.'.format(self.timer))
                break
    
    def get_odor_plume(self):
        queue_length = self.odor_plume_sensor_left.getQueueLength()
        if queue_length > 0:
            for i in range(queue_length-1):
                self.odor_plume_sensor_left.nextPacket()
            d_ = self.odor_plume_sensor_left.getString().split(";")
            self.odor_plume_sensor_left.nextPacket()
            o_l = float(d_[0])
            w_l = np.fromstring(d_[1].strip('[]'), dtype=float, sep=' ')
        else:
            o_l = None
            w_l = None
        queue_length = self.odor_plume_sensor_right.getQueueLength()
        if queue_length > 0:
            for i in range(queue_length-1):
                self.odor_plume_sensor_right.nextPacket()
            d_ = self.odor_plume_sensor_right.getString().split(";")
            self.odor_plume_sensor_right.nextPacket()
            o_r = float(d_[0])
            w_r = np.fromstring(d_[1].strip('[]'), dtype=float, sep=' ')
        else:
            o_r = None
            w_r = None
        return o_l, w_l, o_r, w_r
    
    def odor_plume_tracking(self, start_position=None, start_heading=None, timeout=10, num_loop=1,
                            show_trajectory=True, trajectory_color=0x00ff00, trajectory_density=0.8):
        interval_step = self.prepare_navigation(start_position, start_heading,
                                                show_trajectory, trajectory_color, trajectory_density)
        for _ in range(int(timeout*1000/self.time_step)):
            o_l, w_l, o_r, w_r = self.get_odor_plume()
            no_odor = True
            d = 0
            dir_ = 0
            print(f'left: odor {o_l}, wind {w_l}', f"right: odor {o_r}, wind {w_r}")
            if o_l is not None:
                if o_l > 1e-6 or o_r > 1e-6:
                    no_odor = False
                    # if odor is stronger on the left
                    if o_l > o_r:
                        # turn upwind defined by the wind direction at left antenna
                        dir_ = np.arctan2(w_l[1], w_l[0])
                    else:
                        # turn upwind defined by the wind direction at right antenna
                        dir_ = np.arctan2(w_r[1], w_r[0]) 
    
                    h = self.get_self_rotation()[-1] + np.pi * (self.get_self_rotation()[-2]<0)
                    h = pi2pi(h)
                    dir_ = pi2pi(dir_)
                    # d = pi2pi(dir_ - h) / np.deg2rad(13)
                    d = pi2pi(dir_ - h)
                    print('dir', np.rad2deg(dir_), 'h', np.rad2deg(h), 'd', np.rad2deg(d))
                    # TODO:use average wind direction to turn
            
            if no_odor:
                o_l = 0
                o_r = 0
                w_l = np.array([0, 0])
                w_r = np.array([0, 0])
                self.walker.parameters['HipSwing'] = np.deg2rad(5)
                self.walker.parameters['Rotation'] = np.random.choice([-1, 1])
                self.walker.generate_sequence()
                n = 1
                # self.walk(1)
            else:
                # if d > 0:
                #     d = np.min([d, 6])
                # else:
                #     d = np.max([d, -6])
                # rotate
                sw = np.min([5 + abs(np.rad2deg(d)//5), 15])
                self.walker.parameters['HipSwing'] = np.deg2rad(sw)
                # self.walker.parameters['HipSwing'] = np.deg2rad(int(np.abs(dir)))
                self.walker.parameters['Rotation'] = int(np.sign(d))
                # self.walker.parameters['Rotation'] = int(-dir)
                self.walker.generate_sequence()
                self.walk(1)
                # n = int(abs(d))
                # go forward
                self.walker.parameters['HipSwing'] = np.deg2rad(15)
                self.walker.parameters['Rotation'] = 0
                self.walker.generate_sequence()
                # self.walk(2)
                n = 2
            
            for _ in range(self.walker.sequence_len*n):
                self.walker.set_pose_from_sequence()
                self.my_step()
                self.logger.data['time'].append(self.getTime())
                
                self.logger.data['odor_l'].append(o_l)
                self.logger.data['odor_r'].append(o_r)
                self.logger.data['wind_l'].append(w_l.tolist())
                self.logger.data['wind_r'].append(w_r.tolist())
                self.logger.data['position'].append(self.get_self_position())
                self.logger.data['heading'].append(self.get_self_rotation()[-1])
                self.logger.data['velocity'].append(self.get_self_velocity())
                self.logger.data['angular_velocity'].append(self.get_self_angular_velocity())
            
            if self.get_self_position()[0] < -2.8:
                self.logger.log('Odor plume tracking finished at t={}.'.format(self.timer))
                print('Odor plume tracking finished at t={}.'.format(self.timer))
                break
            # print('odor:', not no_odor, 'turning', d)
