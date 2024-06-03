import json
import os
import datetime
import matplotlib.pyplot as plt
import numpy as np
from constant import *
from math import factorial
import scipy.io as sio

def calculate_speed(velocity):
    return np.sqrt(velocity[:,0]**2 + velocity[:,1]**2)

def calculate_velocity(data, end_time=480):
    velocity = []
    interval = 80
    t = np.array(data['time'])[:end_time]
    p = np.array(data['position'])[:end_time]
    for i in range(1, len(t)//interval):
        dt = t[i*interval] - t[(i-1)*interval]
        dx = p[i*interval][0] - p[(i-1)*interval][0]
        dy = p[i*interval][1] - p[(i-1)*interval][1]
        # dz = data['position'][i+interval][2] - data['position'][i][2]
        velocity.append([dx/dt, dy/dt])
    return t[::interval][:len(velocity)], velocity

def pi2pi(direction):
    return (direction + np.pi) % (2 * np.pi) - np.pi


def visual_world_mat2obj(mat_filename, obj_filename):
    w = sio.loadmat(mat_filename)
    with open(obj_filename, 'w') as f:
        for i in range(w['X'].shape[0]):
            for j in range(3):
                f.write(f"v {w['X'][i, j]} {w['Y'][i, j]} {w['Z'][i, j]}\n")
            # f.write(f"v {w1['X'][i, 0]} {w1['Y'][i, 0]} {w1['Z'][i, 0]}\n")
        for i in range(w['X'].shape[0]):
            s = i * 3
            f.write(f"f {s+1} {s+2} {s+3}\n")


class Logger:
    def __init__(self, path, name):
        self.path = path
        # clear log file
        with open(self.path + '{}_log.txt'.format(name), 'w') as f:
            f.write('')
        self.data = {}
        self.name = name

    def log(self, message):
        fp = self.path + '{}_log.txt'.format(self.name)
        with open(fp, 'a') as f:
            f.write("*{}: {}".format(
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 
                message) + '\n')
    
    def save(self):
        fp = self.path + '{}_data.json'.format(self.name)
        with open(fp, 'w') as f:
            json.dump(self.data, f, indent=2)


class Analyzer:
    def __init__(self, data_file) -> None:
        with open(data_file, 'r') as f:
            self.data = json.load(f)
    

class Plotter:
    def __init__(self, data) -> None:
        self.fontsize_s = 12
        self.fontsize_m = 16
        self.fontsize_l = 20
        self.data = data
    
    def plot_leg_joint(self, leg='FL'):
        fig, ax = plt.subplots(figsize=(8, 4))
        name = 'ps-Leg-' + leg
        for i in range(5):
            ax.plot(self.data['time'], np.array(self.data[name])[:,i],
                    color=LEG_SEG_COLOR[i],
                    label=LEG_JOINT_NAME[i])
        # add swing and stance phase
        t_data = self.data['ts-Leg-{}'.format(leg)]
        mask = np.array(t_data) == 1
        ax.fill_between(self.data['time'], -2, -2 + mask*5, color='k', step='mid', alpha=0.1)
        ax.set_ylim(-1, 2.5)
        ax.legend(fontsize=self.fontsize_s)
        ax.set_xlabel('Time (s)',fontszie=self.fontsize_m)
        ax.set_ylabel('Position (rad)',fontsize=self.fontsize_m)
        ax.tick_params(axis='both', which='major', labelsize=self.fontsize_s)
        
        return fig, ax


class ZernikeMoment:
    def __init__(self, n_max=60, m_max=20, size=128) -> None:
        self.n_max = n_max
        self.m_max = m_max
        self.p_max = int(self.n_max/2)
        self.q_max = int((self.m_max + self.n_max)/2)
        self.size = size
        
        self.pre_calc_factorials()
        self.pre_calc_polar()
    
    def pre_calc_factorials(self):
        self.fac_s = np.array([factorial(i) for i in range(self.p_max + 1)])
        self.fac_n_s = np.zeros([self.n_max, self.p_max + 1])
        for n in range(self.n_max):
            for s in range(int(n/2) + 1):
                self.fac_n_s[n, s] = factorial(n - s)
        self.fac_q_s = np.zeros([self.q_max, self.p_max + 1])
        for q in range(self.q_max):
            for s in range(np.min([q+1, self.p_max+1])):
                self.fac_q_s[q, s] = factorial(q - s)
        self.fac_p_s = np.zeros([self.p_max, self.p_max + 1])
        for p in range(self.p_max):
            for s in range(p + 1):
                self.fac_p_s[p, s] = factorial(p - s)
    
    def pre_calc_polar(self):
        x = range(self.size)
        y = x
        p_x, p_y = np.meshgrid(x, y)
        self.p_r = np.sqrt((2*p_x - self.size + 1)**2 + (2*p_y - self.size + 1)**2) / self.size
        self.p_theta = np.arctan2(self.size - 1 - 2*p_y, 2*p_x - self.size + 1)
        self.p_r = np.where(self.p_r <= 1, 1, 0) * self.p_r
        
    def radial_poly(self, r, n, m):
        rad = np.zeros(r.shape, r.dtype)
        P = int((n - abs(m)) / 2)
        Q = int((n + abs(m)) / 2)
        for s in range(P + 1):
            c = (-1) ** s * self.fac_n_s[n,s]
            c /= self.fac_s[s] * self.fac_q_s[Q,s] * self.fac_p_s[P,s]
            rad += c * r ** (n - 2 * s)
        return rad
    
    
    def compute(self, src, n, m):
        if src.dtype != np.float32:

            src = 1.0 - src.astype(np.float32)/255.0
        if len(src.shape) == 3:
            print('the input image src should be in gray')
            return

        # get the radial polynomial
        Rad = self.radial_poly(self.p_r, n, m)

        Product = src * Rad * np.exp(-1j * m * self.p_theta)
        # calculate the moments
        pc_Z = Product.sum()

        # count the number of pixels inside the unit circle
        cnt = np.count_nonzero(self.p_r) + 1
        # normalize the amplitude of moments
        z = (n + 1) * pc_Z / cnt
        # calculate the amplitude of the moment
        a = abs(pc_Z)
        # calculate the phase of the moment (in degrees)
        p = np.angle(pc_Z) * 180 / np.pi

        return z, a, p