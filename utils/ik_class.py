#! /usr/bin/env python
# Class definitions for the kinematics of the serial-2R manipulator
# and the modified five-bar manipulator (Stoch-2 leg)
#
#
# Created : 17 Feb, 2021
# Author: Tejas, Aditya, Shishir, Chandravaran
# 

import numpy as np
import math
from scipy import optimize

PI = np.pi

# Serial2R Kinematics class
# Functions include : Forward kinematics, inverse kinematics, Jacobian w.r.t the end-effector
# Assumes absolute angles between the links

class Serial_RRP_Kinematics():


    def __init__(self, 
            base_pivot=[0,0], 
            link_lengths=[0.172,0.125]):
        self.link_lengths = link_lengths
        self.base_pivot = np.array([0,-0.15])
        self.delta_dt=0.01

        self.dt=0.0005*2
        self.frequency = 4
        self.omega = 2 * np.pi * self.frequency
        # self.omega=4.3982
        self.d_phase=self.dt*self.omega
        
        self.q0=[]
        self.q1=[]
        self.q2=[]
        self.q0.append(1.2) #initial rad position
        self.q1.append(-2.05)
        self.q2.append(0.0)
        # self.a=0.1
        # self.b=0.05402826
        self.f=[]
        self.f.append((1.2,-2.05,0.012))

        # self.b=0.026
        # self.a=0.00215       

    # def cosineRule(self, a, b, c):
    #     '''
    #     Cosine Rule implementation for triangle sides a, b, c
    #     cos A
    #     '''
    #     return math.acos((c**2 + b**2 - a**2)/(2*b*c))
    def traj_gen(self,phase):

        # for 5 cm desired jump
        z=-0.174-0.026*np.cos(phase)


        # for 10 cm desired jump
        # z=-0.15+0.05*np.sin(phase)
        # x=-0.0216+0.024*np.cos(phase)
        x=-0.03
        return x,z

    def inverseKinematics(self, phase, branch="<"):

        valid = True
        [l1, l2] = self.link_lengths
        # t=phase
        
        # t_iteration=t/(0.02*PI)

        # angle_1=self.q0[len(self.q0)-1]
        # angle_2=self.q1[len(self.q1)-1]
        # h3=0.012-self.q2[len(self.q2)-1]
        '''
        # j=np.zeros((2,3))
        # j[0][0]=-l1*np.cos(angle_1)-l2*np.cos(angle_1+angle_2)-h3*np.cos(angle_1+angle_2)
        # j[0][1]=-l2*np.cos(angle_1+angle_2)-h3*np.cos(angle_1+angle_2)
        # j[0][2]=-np.sin(angle_1+angle_2)
        # j[1][0]=l1*np.sin(angle_1)+l2*np.sin(angle_1+angle_2)+h3*np.sin(angle_1+angle_2)
        # j[1][1]=l2*np.sin(angle_1+angle_2)+h3*np.sin(angle_1+angle_2)
        # j[1][2]=-np.cos(angle_1+angle_2)
        # # print(j.shape)
        # m=np.linalg.lstsq(np.dot(j.T,j),np.array([[1,0,0],[0,1,0],[0,0,1]]))[0]  #
        # pseudo_j = np.dot(m,j.T)
        # des_pos=np.array([self.a*np.cos(phase),self.b*np.sin(phase)])
        # d_pos=np.array([self.a*np.cos(phase+self.d_phase) - self.a*np.cos(phase),
        #                 self.b*np.sin(phase+self.d_phase) - self.b*np.sin(phase)]) #different in pos
        # delta_q=np.dot(pseudo_j,d_pos.T)
        # self.q0.append(self.q0[len(self.q0)-1]+delta_q[0])
        # self.q1.append(self.q1[len(self.q1)-1]+delta_q[1])
        # if self.q2[len(self.q2)-1]-delta_q[2]>0.01200 :
        #     self.q2.append(0.012)
        # elif self.q2[len(self.q2)-1]-delta_q[2]<0.000 :
        #     self.q2.append(0.0)
        # else :
        #     self.q2.append(self.q2[len(self.q2)-1]-delta_q[2])
        '''

        x0=self.f[len(self.f)-1]
        
        m,n=self.traj_gen(phase)
        
        ans=optimize.fmin_slsqp(self.objective,x0,eqcons=[self.eqcon1,self.eqcon2],bounds=[(0.0,2.00),(-3.14,2.00),(0.0,0.012)],args=(m,n))
        
        self.f.append(ans)


        valid = True
        return valid, np.array([self.f[len(self.f)-1][0],self.f[len(self.f)-1][1],0.054-self.f[len(self.f)-1][2]])


    def forwardKinematics(self, q):

        [l1, l2] = self.link_lengths

        # q[2]=0.012-q[2]
        q[2]=0.054-q[2]
        x = self.base_pivot + l1*np.array([-math.sin(q[0]), -math.cos(q[0])]) + l2*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])]) + q[2]*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])])
        #x = self.base_pivot + l1*np.array([-math.sin(q[0]), -math.cos(q[0])]) + l2*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])]) + (q[2]+0.05)*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])])

        return x
    
    def forward_kin_actual(self,q,b=None):

        if b is None:
            b=self.base_pivot
        [l1, l2] = self.link_lengths

        # q[2]=0.012-q[2]
        q[2]=0.054-q[2]
        x = b + l1*np.array([-math.sin(q[0]), -math.cos(q[0])]) + l2*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])]) + (q[2])*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])])
        #x = b + l1*np.array([-math.sin(q[0]), -math.cos(q[0])]) + l2*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])]) + (q[2]+0.05)*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])])

        return x 

    def objective(self,a,m,n):

        return (a[0]**2 + a[1]**2 + a[2]**2)

    def eqcon1(self,a,m,n):
        [l1,l2]=self.link_lengths

        return (n+l1*np.cos(a[0])+l2*np.cos(a[0]+a[1])+a[2]*np.cos(a[0]+a[1]))

    def eqcon2(self,a,m,n):
        [l1,l2]=self.link_lengths

        return (m+l1*np.sin(a[0])+l2*np.sin(a[0]+a[1])+a[2]*np.sin(a[0]+a[1]))

    def Jacobian(self, q):

        [l1, l2] = self.link_lengths

        angle_1=q[0]
        angle_2=q[1]
        # h3=0.012-q[2]
        h3=0.054-q[2]
        #h3=0.012+0.05-q[2]


        j=np.array([[-l1*np.cos(angle_1)-l2*np.cos(angle_1+angle_2)-h3*np.cos(angle_1+angle_2),
                        -l2*np.cos(angle_1+angle_2)-h3*np.cos(angle_1+angle_2),
                        -np.sin(angle_1+angle_2)],

                    [l1*np.sin(angle_1)+l2*np.sin(angle_1+angle_2)+h3*np.sin(angle_1+angle_2),
                        l2*np.sin(angle_1+angle_2)+h3*np.sin(angle_1+angle_2),
                        -np.cos(angle_1+angle_2)]])        

        return j





"""
if __name__ == '__main__':
    #s = Serial2RKin([0,0],[0.15,0.175])
    s = StochliteKinematics()
    valid, angles = s.inverseKinematics("FR", [0, -0.096,-0.317999999])
    if valid:
        print(angles)
    else:
        print("invalid")
    angles = np.array([-1.5705246792692433, -1.5695507528642707, 0.829469970861002])
    cordinates = s.forwardKinematics("FR", angles)
    print(cordinates)
"""

#End of file

# if __name__ == '__main__':
#     s=Serial_RRP_Kinematics()
#     phase=0.00
#     _,q=s.inverseKinematics(phase)
#     x,z=s.traj_gen(phase)
#     print("x-coordinate=",x)
#     print("z-coordinate=",z)
#     print("Hip Angle=",q[0])
#     print("Knee Angle=",q[1])
