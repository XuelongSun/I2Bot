import numpy as np
import random
from scipy.special import expit

def gen_tb_tb_weights(weight=1.):
    """Weight matrix to map inhibitory connections from TB1 to other neurons"""
    W = np.zeros([8, 8])
    sinusoid = -(np.cos(np.linspace(0, 2 * np.pi, 8, endpoint=False)) - 1) / 2
    for i in range(8):
        values = np.roll(sinusoid, i)
        W[i, :] = values
    return weight * W


def noisy_sigmoid(v, slope=1.0, bias=0.5, noise=0.01):
    """Takes a vector v as input, puts through sigmoid and
    adds Gaussian noise. Results are clipped to return rate
    between 0 and 1"""
    sig = expit(v * slope - bias)
    if noise > 0:
        sig += np.random.normal(scale=noise, size=len(v))
    return np.clip(sig, 0, 1)


def sigmoid(x, deriv=False):
    if (deriv == True):
        return x * (1 - x)
    return 1 / (1 + np.exp(-x))

class CentralComplexModel(object):
    """Class for the CX of current heading (PB), cue integration (FB) and steering circuit (FB).
       This implementation is adapted from Stone et.al 2017, https://doi.org/10.1016/j.cub.2017.08.052
    """
    def __init__(self):
        # global current heading
        # tl2 neurons
        self.tl2_prefs = np.tile(np.linspace(0, 2 * np.pi, 8, endpoint=False), 2)
        self.tl2 = np.zeros(8)
        # cl1 neurons
        self.cl1 = np.zeros(8)
        # I_tb1 neurons
        self.I_tb1 = np.zeros(8)
        # connection weights
        # cl1 -> I_tb1
        self.W_CL1_TB1 = np.tile(np.eye(8), 2)
        # I_tb1 -> I_tb1
        self.W_TB1_TB1 = gen_tb_tb_weights()

        # local current heading
        self.phase_prefs = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        # local compass neuron - II_tb1
        self.II_tb1 = np.zeros(8)

        # steering circuit
        self.desired_heading_memory = np.zeros(16)
        self.current_heading_memory = np.zeros(8)
        # pre-motor neuron CPU1
        self.cpu1a = np.zeros(14)
        self.cpu1b = np.zeros(2)
        self.cpu1 = np.zeros(16)
        # motor[0] for left and motor[1] for right
        self.motor = np.zeros(2)
        # positive -> turn left, negative -> turn right
        self.motor_value = 0
        # connection weights from CPU1 to motor
        self.W_CPU1a_motor = np.array([
            [1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1]])
        self.W_CPU1b_motor = np.array([[0, 1],
                                       [1, 0]])

        # connection weights to steering circuit
        # current heading -> steering circuit
        self.W_CH_CPU1a = np.tile(np.eye(8), (2, 1))[1:14 + 1, :]
        self.W_CH_CPU1b = np.array([[0, 0, 0, 0, 0, 0, 0, 1],
                                    [1, 0, 0, 0, 0, 0, 0, 0]])

        # desired heading -> steering circuit
        self.W_DH_CPU1a = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        ])

        self.W_DH_CPU1b = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 8
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],  # 9
        ])

        # other para
        self.noise = 0.0

    def global_current_heading(self, theta):
        # tl2
        output = np.cos(theta - self.tl2_prefs)
        self.tl2 = noisy_sigmoid(output, 6.8, 3.0, self.noise)
        # cl1
        self.cl1 = noisy_sigmoid(-self.tl2, 3.0, -0.5, self.noise)
        # I_tb1
        prop_cl1 = 0.667  # Proportion of input from CL1 vs TB1
        prop_I_tb1 = 1.0 - prop_cl1
        output = (prop_cl1 * np.dot(self.W_CL1_TB1, self.cl1) -
                  prop_I_tb1 * np.dot(self.W_TB1_TB1, self.I_tb1))
        self.I_tb1 = noisy_sigmoid(output, 5.0, 0.0, self.noise)

        return self.I_tb1

    def local_current_heading(self, phase):
        vc_mem_ring = np.cos(np.deg2rad(phase) - self.phase_prefs)
        self.II_tb1 = 1 / (1 + np.exp(-vc_mem_ring * 3 - 1.0))
        return self.II_tb1

    def steering_circuit_out(self):
        inputs = np.dot(self.W_DH_CPU1a, self.desired_heading_memory) * np.dot(self.W_CH_CPU1a,
                                                                               1.0 - self.current_heading_memory)
        self.cpu1a = noisy_sigmoid(inputs, 5.0, 2.5, self.noise)

        inputs = np.dot(self.W_DH_CPU1b, self.desired_heading_memory) * np.dot(self.W_CH_CPU1b,
                                                                               1.0 - self.current_heading_memory)

        self.cpu1b = noisy_sigmoid(inputs, 5.0, 2.5, self.noise)

        self.cpu1 = np.hstack([self.cpu1b[-1], self.cpu1a, self.cpu1b[0]])

        motor = np.dot(self.W_CPU1a_motor, self.cpu1a)
        motor += np.dot(self.W_CPU1b_motor, self.cpu1b)
        self.motor = motor
        self.motor_value = (self.motor[0] - self.motor[1]) * 0.25

        return self.motor_value


class PathIntegration:
    def __init__(self, initial_memory=0.5):
        self.tn1 = 0
        self.tn2 = 0
        self.initial_memory = initial_memory
        self.memory_gain = 0.04
        self.memory = np.ones(16) * initial_memory
    
    def get_flow(self, heading, velocity, tn_prefs=np.pi / 4.0):
        """Calculate optic flow depending on preference angles. [L, R]"""
        A = np.array([[np.cos(heading + tn_prefs),
                       np.sin(heading + tn_prefs)],
                      [np.cos(heading - tn_prefs),
                     np.sin(heading - tn_prefs)]])
        flow = np.dot(A, velocity)

        return flow

    def update_neuron_activation(self, cx:CentralComplexModel, heading, velocity):
        # optic flow and the activation of TN1 and TN2 neurons
        flow = self.get_flow(heading, velocity)
        output = (1.0 - flow) / 2.0
        if cx.noise > 0.0:
            output += np.random.normal(scale=cx.noise, size=flow.shape)
        self.tn1 = np.clip(output, 0.0, 1.0)
        output = flow
        if cx.noise > 0.0:
            output += np.random.normal(scale=cx.noise, size=flow.shape)
        self.tn2 = np.clip(output, 0.0, 1.0)

        # CPU4
        mem_reshaped = self.memory.reshape(2, -1)
        mem_update = (0.5 - self.tn1.reshape(2, 1)) * (1.0 - cx.I_tb1)
        mem_update -= 0.5 * (0.5 - self.tn1.reshape(2, 1))
        mem_reshaped += self.memory_gain * mem_update
        self.memory = np.clip(mem_reshaped.reshape(-1), 0.0, 1.0)
        return self.memory


class VisualBeacon:
    def __init__(self) -> None:
        self.output = None
        self.vh_k = 0.2
    
    def update_neuron_activation(self, img_l, img_r, cx:CentralComplexModel, mode='left', thr=15):
        if mode == 'left':
            roi = np.where(img_l<=thr)
            if len(roi[0]) > 0:
                delta = img_l.shape[1]/2 - np.mean(roi[1])
            else:
                delta = 0.02*(np.random.rand() - 0.5)
        elif mode == 'right':
            roi = np.where(img_r<=thr)
            if len(roi[0]) > 0:
                delta = img_r.shape[1]/2 - np.mean(roi[1])
            else:
                delta = 0.02*(np.random.rand() - 0.5)
        elif mode == 'both':
            roi_l = np.where(img_l<=thr)
            roi_r = np.where(img_r<=thr)
            if len(roi_l[0]) > 0:
                delta_l = img_l.shape[1]/2 - np.mean(roi_l[1])
            else:
                delta_l = 0.02*(np.random.rand() - 0.5)
            if len(roi_r[0]) > 0:
                delta_r = img_r.shape[1]/2 - np.mean(roi_r[1])
            else:
                delta_r = 0.02*(np.random.rand() - 0.5)
            delta = (delta_l + delta_r)/2
        # shift = int(np.min([np.max([abs(int(delta * self.vh_k)), 0]), 3])*np.sign(delta))
        shift = np.ceil((abs(6 * delta / img_l.shape[1])))
        vb = np.roll(cx.I_tb1, int(shift* np.sign(delta)))
        vb = expit(vb * 5.0 - 2.5)
        self.output = vb
        # print('s', shift, 'd', delta)
        return self.output
    