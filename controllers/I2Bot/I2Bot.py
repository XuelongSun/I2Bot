from robots import *

# demo of ['gait', 'PI', 'VisualBeacon', 'VisionCompass', 'OdorPlumeTracking', 'OdorTrialFollowing']
demo = 'VisualBeacon'

if demo == 'gait':
    # add_force = 130
    
    # agent.adhesion_force = add_force
    # agent = GaitController('ik_gait')
    # agent.enable_adhesion = False
    agent = GaitController(f'fk_wave_gait_uneven_test')
    agent.walker_fk.parameters = {
        'Gait':'Wave',
        'HipSwing': 0.2,
        'LiftSwing': 0.4,
        'StepNum':20,
        'Rotation':0,
        'Direction':1
    }
    agent.walker_ik.parameters = {
        'Gait':'Tripod',
        'Length': 0.02,
        'Height': 0.02,
        'StepNum':20,
        'Rotation':0
    }
    # set the walker type
    # agent.set_walker_type('FK')
    agent.set_walker_type('IK')

    # record_para = {
    #     'video': True,
    #     'snapshots': [10, 220, 420],
    #     'record_file_path': 'records/'
    # }
    record_para = {
        'video': False,
        'snapshots': [],
        'record_file_path': 'records/'
    }
    agent.walk(40, record_para)
    agent.logger.save()
else:
    if demo == 'PI':
        nest_p = [0., 0.]
        # heading = np.random.uniform(-np.pi, np.pi)
        heading = 1.5
        # agent = NavigationController('PI_h={:.2f}'.format(np.rad2deg(heading)))
        agent = NavigationController('PI_test')
        agent.pi_foraging(nest_p, heading, timeout=32)
        agent.motor_k = 5
        agent.gait_ctl_loop_num = 1
        # generate random initial positions within x: [-20, 20], y: [-20, 20]
        # x = np.random.uniform(-20, 20)
        # y = np.random.uniform(-20, 20)
        # generate initial heading within [-pi, pi] according to the random initial position
        agent.pi_homing(timeout=200)
        agent.logger.save()
    elif demo == 'VisualBeacon':
        agent = NavigationController('VisualBeaconNewTest')
        mode = 'left'
        if mode == 'left':
            h = np.random.uniform(-np.pi/2, 0)
        elif mode == 'right':
            h = np.random.uniform(0, np.pi/2)
        elif mode == 'both':
            h = np.random.uniform(-np.pi/2, np.pi/2)
        agent.motor_k = 0.5
        agent.gait_ctl_loop_num = 1
        agent.visual_beacons(start_heading=h+np.pi/2, timeout=100, mode=mode)
        agent.logger.save()
    elif demo == 'VisionCompass':
        agent = NavigationController('VisualCompass_test')
        agent.visual_compass(vs_mode='jump', timeout=120)
        agent.logger.save()
    elif demo == 'OdorPlumeTracking':
        agent = NavigationController('OdorPlumeTest')
        # agent.wait(2000)
        agent.odor_plume_tracking()
        agent.logger.save()
    elif demo == 'OdorTrialFollowing':
        agent = NavigationController('OdorTrialFollowingTest')
        agent.odor_trial_following(antenna_mode='move')
        agent.logger.save()
    else:
        print('Invalid demo name!')