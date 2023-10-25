import numpy as np
import matplotlib.pyplot as plt
from gym import spaces
from collections import deque
import time
import math as m
PI = np.pi

def __init__(self):
    self.dt = 0.008
    self.frequency = 0.70
    self.omega = 2 * np.pi * self.frequency
    
    self.leg_step_length= 0.15
    self.step_height = 0.075  
    self.z_foot = -0.2

    self.phase = 0
    self.time_step = 0
    self.all_phases = np.linspace(0, 2*np.pi, num=200)


    self.all_desired_pos = []
    
    for phase in self.all_phases:
        desired_pos, _ = self.calc_desired_pos(phase)
        self.all_desired_pos.append(desired_pos)

    self.all_desired_pos = np.array(self.all_desired_pos)


def cosineRule(self, a, b, c):
    '''
    Cosine Rule implementation for triangle sides a, b, c
    cos A
    '''
    return m.acos((c**2 + b**2 - a**2)/(2*b*c))

def inverseKinematics(self, path, branch="<"):
    '''
    Inverse kinematics of a serial 2-R manipulator

    Note - Leg is in x-z plane, rotation about y

    Inputs:
    -- base_pivot: Position of the base pivot (in Cartesian co-ordinates)
    -- link_len: Link lenghts [l1, l2]
    -- ee_pos: position of the end-effector [x, y] (Cartesian co-ordinates)

    Output:
    -- Solutions to both the branches of the IK. Angles specified in radians.
    -- Note that the angle of the knee joint is relative in nature.
-- Note the hip is taken with respective to the positive x- axis 
    '''

    valid = True
    q = np.zeros(2, float)
    [x,z]=np.array(path)
    [l1, l2] = self.link_lengths
    # Check if the end-effector point lies in the workspace of the manipulator
    if ((x**2 + z**2) > (l1+l2)**2) or ((x**2 + z**2) < (l1-l2)**2):
        #print("Point is outside the workspace")
        valid=False
        return valid, q

    r = m.sqrt(x**2 + z**2)
    t1 = m.atan2(-z, -x)
    
    q[0] = PI/2 - t1 - self.cosineRule(l2, r, l1)
    q[1] = PI - self.cosineRule(r, l1, l2)

    if branch == "<":
        q[0] = PI - 2*t1 - q[0]
        q[1] = q[1] * -1

    valid = True
    return valid, q




def get_pos(self):
    
    phase = self.phase + self.omega*self.dt*self.action_repeat

    self.desired_pos= self.calc_desired_pos(phase)
    _,self.desired_angles = inverseKinematics(self.desired_pos,branch='<')


def calc_desired_pos(self, phase=None):

    if phase is None:
        phase = self.phase

    if phase < np.pi:
        x = self.leg_step_length * np.cos(phase)
        z = self.z_foot

    else:
        x = self.leg_step_length * np.cos(2*np.pi - phase)
        z = self.z_foot + self.step_height * np.sin(2*np.pi - phase)


    return np.array([x, z])