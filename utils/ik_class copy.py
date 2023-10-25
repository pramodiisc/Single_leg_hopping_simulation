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

PI = np.pi

# Serial2R Kinematics class
# Functions include : Forward kinematics, inverse kinematics, Jacobian w.r.t the end-effector
# Assumes absolute angles between the links

class Serial2RKinematics():
    def __init__(self, 
            base_pivot=[0,0], 
            link_lengths=[0.146,0.172]):
        self.link_lengths = link_lengths
        self.base_pivot = base_pivot

    def cosineRule(self, a, b, c):
        '''
        Cosine Rule implementation for triangle sides a, b, c
        cos A
        '''
        return math.acos((c**2 + b**2 - a**2)/(2*b*c))

    def inverseKinematics(self, ee_pos, branch="<"):
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
        x_z_points = np.array(ee_pos) - np.array(self.base_pivot)
        [x, z] = x_z_points.tolist() 

        [l1, l2] = self.link_lengths
        # Check if the end-effector point lies in the workspace of the manipulator
        # if ((x**2 + z**2) > (l1+l2)**2) or ((x**2 + z**2) < (l1-l2)**2):
        #     #print("Point is outside the workspace")
        #     valid=False
        #     return valid, q

        r = math.sqrt(x**2 + z**2)
        t1 = math.atan2(-z, -x)
        
        q[0] = PI/2 - t1 - self.cosineRule(l2, r, l1)
        q[1] = PI - self.cosineRule(r, l1, l2)

        if branch == "<":
            q[0] = -1* q[0]
            q[1] = q[1] * -1

        valid = True
        return valid, q
    

    def forwardKinematics(self, q):
        '''
        Forward Kinematics of the serial-2R manipulator

        Note - Leg is in x-z plane, rotation about y

        Args:
        --- q : A vector of the joint angles [q_hip, q_knee], where q_knee is relative in nature
        Returns:
        --- x : The position vector of the end-effector
        '''
        [l1, l2] = self.link_lengths
        x = self.base_pivot + l1*np.array([-math.sin(q[0]), -math.cos(q[0])]) + l2*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])])
        # x = self.base_pivot + l1*np.array([-math.cos(q[0]), -math.cos(q[0])]) + l2*np.array([-math.cos(q[0] - q[1]), -math.cos(q[0] - q[1])])
        return x


    def Jacobian(self, q):
        '''
        Provides the Jacobian matrix for the end-effector
        Args:
        --- q : The joint angles of the manipulator [q_hip, q_knee], where the angle q_knee is specified relative to the thigh link
        Returns:
        --- mat : A 2x2 velocity Jacobian matrix of the manipulator
        '''
        [l1, l2] = self.link_lengths
        mat = np.zeros([2,2])
        mat[0,0] = -l1*math.cos(q[0]) - l2*math.cos(q[0] + q[1])
        mat[0,1] = -l2*math.cos(q[0] + q[1])
        mat[1,0] = l1*math.sin(q[0]) + l2*math.sin(q[0] + q[1])
        mat[1,1] = l2*math.sin(q[0] + q[1])
        return mat

class Serial_RRP_Kinematics():


    def __init__(self, 
            base_pivot=[0,0], 
            link_lengths=[0.146,0.172]):
        self.link_lengths = link_lengths
        self.base_pivot = np.array([0,0])
        self.delta_dt=0.01

        self.dt=0.0005*2
        self.omega=4.3982
        self.d_phase=self.dt*self.omega
        
        self.q0=[]
        self.q1=[]
        self.q2=[]
        self.q0.append(1.30) #initial rad position
        self.q1.append(-2.2)
        self.q2.append(0.0)
        self.a=0.1
        self.b=0.05402826
        

    def cosineRule(self, a, b, c):
        '''
        Cosine Rule implementation for triangle sides a, b, c
        cos A
        '''
        return math.acos((c**2 + b**2 - a**2)/(2*b*c))

    # def inverseKinematics(self, phase, branch="<"):

    #     valid = True
        
    #     # x_z_points = np.array(ee_pos) - np.array(self.base_pivot)
    #     # [x, z] = x_z_points.tolist() 
    #     # phase=math.atan2(z+0.31166117,x+0.0051654)/(2*PI)
    #     [l1, l2] = self.link_lengths
        


    #     # if ((x**2 + z**2) > (l1+l2)**2) or ((x**2 + z**2) < (l1-l2)**2):
    #     #     #print("Point is outside the workspace")
    #     #     valid=False
    #     #     return valid, q

    #     # r = math.sqrt(x**2 + z**2)
    #     # t1 = math.atan2(-z, -x)

    #     t=phase
        
    #     t_iteration=t/(0.02*PI)
        

    #     # t_plus_dt_constraint=(t+self.delta_dt)%(2*np.pi)
    #     # t_plus_dt_constraint=time+self.delta_dt
    #     #print("t_plus_dt_constraint", t_plus_dt_constraint)
    #     # t_constraint=t%(2*np.pi)
    #     #t_constraint=time
    #     angle_1=self.q0[len(self.q0)-1]
    #     angle_2=self.q1[len(self.q1)-1]
    #     h3=self.q2[len(self.q2)-1]
    #     j=np.zeros((2,3))
    #     j[0][0]=l1*np.cos(-angle_1)+l2*np.cos(-angle_1-angle_2)+h3*np.cos(-angle_1-angle_2)
    #     j[0][1]=l2*np.cos(-angle_1-angle_2)+h3*np.cos(-angle_1-angle_2)
    #     j[0][2]=np.sin(-angle_1-angle_2)
    #     j[1][0]=-l1*np.sin(-angle_1)-l2*np.sin(-angle_1-angle_2)-h3*np.sin(-angle_1-angle_2)
    #     j[1][1]=-l2*np.sin(-angle_1-angle_2)-h3*np.sin(-angle_1-angle_2)
    #     j[1][2]= np.cos(-angle_1-angle_2)

    #     # j=np.array([[l1*np.cos(-angle_1)+l2*np.cos(-angle_1-angle_2)+h3*np.cos(-angle_1-angle_2),
    #     #                 l2*np.cos(-angle_1-angle_2)+h3*np.cos(-angle_1-angle_2),
    #     #                 np.sin(-angle_1-angle_2)],

    #     #             [-l1*np.sin(-angle_1)-l2*np.sin(-angle_1-angle_2)-h3*np.sin(-angle_1-angle_2),
    #     #                 -l2*np.sin(-angle_1-angle_2)-h3*np.sin(-angle_1-angle_2),
    #     #                 np.cos(-angle_1-angle_2)]])
    #     print(j.shape)
    #     m=np.linalg.lstsq(np.dot(j.T,j),np.array([[1,0,0],[0,1,0],[0,0,1]]))[0]  #
    #     #m1=np.linalg.inv(np.dot(j.T,j))
    #     pseudo_j = np.dot(m,j.T)

    #     d_pos=np.array([-self.a*np.sin(-phase)*0.00439822,
    #                     self.b*np.cos(phase)*0.00439822]) #different in pos

    #     delta_q=np.dot(pseudo_j,d_pos.T)

    #     self.q0.append(self.q0[len(self.q0)-1]-delta_q[0])
    #     self.q1.append(self.q1[len(self.q1)-1]-delta_q[1])
    #     self.q2.append(self.q2[len(self.q2)-1]+delta_q[2])
    #     if self.q2[len(self.q2)-1]<0.0 :
    #         z=self.q2.pop()
    #         self.q2.append(0.0)
    #         kin=Serial2RKinematics([0,0],[0.146,0.184])
    #         _,e=kin.inverseKinematics(np.array([0.00594727-0.1+self.a * np.cos(phase),-0.14597174+self.b*np.sin(phase)]))
    #         x=self.q0.pop()
    #         y=self.q1.pop()
    #         self.q0.append(e[0])
    #         self.q1.append(e[1])
    #         # self.q0[len(self.q0)-1]=e[0]
    #         # self.q1[len(self.q1)-1]=e[1]
    #     if self.q2[len(self.q2)-1]>0.012 :
    #         z=self.q2.pop()
            
    #         self.q2.append(0.012)
    #         kin1=Serial2RKinematics()
    #         _,e1=kin1.inverseKinematics(np.array([0.00594727-0.1+self.a * np.cos(phase),-0.14597174+self.b*np.sin(phase)]))
    #         x=self.q0.pop()
    #         y=self.q1.pop()
    #         self.q0.append(e1[0])
    #         self.q1.append(e1[1])
    #             # self.q0[len(self.q0)-1]=e1[0]
    #             # self.q1[len(self.q1)-1]=e1[1]
    #     # if branch == "<":
    #     #     q[0] = PI - 2*t1 - q[0]
    #     #     q[1] = q[1] * -1

    #     valid = True
    #     return valid, np.array([self.q0[len(self.q0)-1],self.q1[len(self.q1)-1],self.q2[len(self.q2)-1]])


    def inverseKinematics(self, phase, branch="<"):

        valid = True
        [l1, l2] = self.link_lengths
        t=phase
        
        t_iteration=t/(0.02*PI)
        
        # t_plus_dt_constraint=(t+self.delta_dt)%(2*np.pi)
        # t_plus_dt_constraint=time+self.delta_dt
        #print("t_plus_dt_constraint", t_plus_dt_constraint)
        # t_constraint=t%(2*np.pi)
        #t_constraint=time
        angle_1=self.q0[len(self.q0)-1]
        angle_2=self.q1[len(self.q1)-1]
        h3=0.012-self.q2[len(self.q2)-1]



        j=np.zeros((2,3))
        # j[0][0]=l1*np.cos(-angle_1)+l2*np.cos(-angle_1-angle_2)+h3*np.cos(-angle_1-angle_2)
        # j[0][1]=l2*np.cos(-angle_1-angle_2)+h3*np.cos(-angle_1-angle_2)
        # j[0][2]=np.sin(-angle_1-angle_2)
        # j[1][0]=-l1*np.sin(-angle_1)-l2*np.sin(-angle_1-angle_2)-h3*np.sin(-angle_1-angle_2)
        # j[1][1]=-l2*np.sin(-angle_1-angle_2)-h3*np.sin(-angle_1-angle_2)
        # j[1][2]= np.cos(-angle_1-angle_2)


        j[0][0]=-l1*np.cos(angle_1)-l2*np.cos(angle_1+angle_2)-h3*np.cos(angle_1+angle_2)
        j[0][1]=-l2*np.cos(angle_1+angle_2)-h3*np.cos(angle_1+angle_2)
        j[0][2]=-np.sin(angle_1+angle_2)
        j[1][0]=l1*np.sin(angle_1)+l2*np.sin(angle_1+angle_2)+h3*np.sin(angle_1+angle_2)
        j[1][1]=l2*np.sin(angle_1+angle_2)+h3*np.sin(angle_1+angle_2)
        j[1][2]=-np.cos(angle_1+angle_2)

        # j=np.array([[l1*np.cos(-angle_1)+l2*np.cos(-angle_1-angle_2)+h3*np.cos(-angle_1-angle_2),
        #                 l2*np.cos(-angle_1-angle_2)+h3*np.cos(-angle_1-angle_2),
        #                 np.sin(-angle_1-angle_2)],

        #             [-l1*np.sin(-angle_1)-l2*np.sin(-angle_1-angle_2)-h3*np.sin(-angle_1-angle_2),
        #                 -l2*np.sin(-angle_1-angle_2)-h3*np.sin(-angle_1-angle_2),
        #                 np.cos(-angle_1-angle_2)]])
        print(j.shape)
        m=np.linalg.lstsq(np.dot(j.T,j),np.array([[1,0,0],[0,1,0],[0,0,1]]))[0]  #
        #m1=np.linalg.inv(np.dot(j.T,j))
        pseudo_j = np.dot(m,j.T)

        # d_pos=np.array([-self.a*np.sin(-phase)*0.00439822,
        #                 self.b*np.cos(phase)*0.00439822]) #different in pos

        d_pos=np.array([self.a*np.cos(phase+self.d_phase) - self.a*np.cos(phase),
                        self.b*np.sin(phase+self.d_phase) - self.b*np.sin(phase)]) #different in pos

        delta_q=np.dot(pseudo_j,d_pos.T)

        self.q0.append(self.q0[len(self.q0)-1]+delta_q[0])
        self.q1.append(self.q1[len(self.q1)-1]+delta_q[1])
        self.q2.append(self.q2[len(self.q2)-1]-delta_q[2])


        # if self.q2[len(self.q2)-1]<0.0 :
        #     z=self.q2.pop()
        #     self.q2.append(0.0)
        #     kin=Serial2RKinematics([0,0],[0.146,0.184])
        #     _,e=kin.inverseKinematics(np.array([0.00594727-0.1+self.a * np.cos(phase),-0.14597174+self.b*np.sin(phase)]))
        #     x=self.q0.pop()
        #     y=self.q1.pop()
        #     self.q0.append(e[0])
        #     self.q1.append(e[1])
        #     # self.q0[len(self.q0)-1]=e[0]
        #     # self.q1[len(self.q1)-1]=e[1]
        # if self.q2[len(self.q2)-1]>0.012 :
        #     z=self.q2.pop()
            
        #     self.q2.append(0.012)
        #     kin1=Serial2RKinematics()
        #     _,e1=kin1.inverseKinematics(np.array([0.00594727-0.1+self.a * np.cos(phase),-0.14597174+self.b*np.sin(phase)]))
        #     x=self.q0.pop()
        #     y=self.q1.pop()
        #     self.q0.append(e1[0])
        #     self.q1.append(e1[1])
        #         # self.q0[len(self.q0)-1]=e1[0]
        #         # self.q1[len(self.q1)-1]=e1[1]
        # # if branch == "<":
        # #     q[0] = PI - 2*t1 - q[0]
        # #     q[1] = q[1] * -1

        valid = True
        return valid, np.array([self.q0[len(self.q0)-1],self.q1[len(self.q1)-1],self.q2[len(self.q2)-1]])


    def forwardKinematics(self, q,b):
        '''
        Forward Kinematics of the serial-2R manipulator

        Note - Leg is in x-z plane, rotation about y

        Args:
        --- q : A vector of the joint angles [q_hip, q_knee], where q_knee is relative in nature
        Returns:
        --- x : The position vector of the end-effector
        '''
        [l1, l2] = self.link_lengths
        print(b)
        q[2]=0.012-q[2]
        x = self.base_pivot + l1*np.array([-math.sin(q[0]), -math.cos(q[0])]) + l2*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])]) + q[2]*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])])

        # x = b + l1*np.array([-math.sin(q[0]), -math.cos(q[0])]) + l2*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])]) + q[2]*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])])
        # x = self.base_pivot + l1*np.array([-math.cos(q[0]), -math.cos(q[0])]) + l2*np.array([-math.cos(q[0] - q[1]), -math.cos(q[0] - q[1])])
        # x = self.base_pivot + l1*np.array([-math.sin(q[0]), -math.cos(q[0])]) + l2*np.array([-math.sin(q[0] + q[1]), -math.cos(q[0] + q[1])])
        return x


    def Jacobian(self, q):
        '''
        Provides the Jacobian matrix for the end-effector
        Args:
        --- q : The joint angles of the manipulator [q_hip, q_knee], where the angle q_knee is specified relative to the thigh link
        Returns:
        --- mat : A 2x2 velocity Jacobian matrix of the manipulator
        '''
        [l1, l2] = self.link_lengths



        """
        angle_1=-q[0]
        angle_2=-q[1]
        h3=q[2]

        # mat = np.zeros([2,2])
        # mat[0,0] = -l1*math.cos(q[0]) - l2*math.cos(q[0] + q[1])
        # mat[0,1] = -l2*math.cos(q[0] + q[1])
        # mat[1,0] = l1*math.sin(q[0]) + l2*math.sin(q[0] + q[1])
        # mat[1,1] = l2*math.sin(q[0] + q[1])

        # j=np.array([[-l1*np.sin(angle_1)-l2*np.sin(angle_1+angle_2)-h3*np.sin(angle_1+angle_2),
        #                 -l2*np.sin(angle_1+angle_2)-h3*np.sin(angle_1+angle_2),
        #                 np.cos(angle_1+angle_2)],

        #             [l1*np.cos(angle_1)+l2*np.cos(angle_1+angle_2)+h3*np.cos(angle_1+angle_2),
        #                 l2*np.cos(angle_1+angle_2)+h3*np.cos(angle_1+angle_2),
        #                 np.sin(angle_1+angle_2)]])
        j=np.array([[l1*np.cos(angle_1)+l2*np.cos(angle_1+angle_2)+h3*np.cos(angle_1+angle_2),
                        l2*np.cos(angle_1+angle_2)+h3*np.cos(angle_1+angle_2),
                        np.sin(angle_1+angle_2)],

                    [-l1*np.sin(angle_1)-l2*np.sin(angle_1+angle_2)-h3*np.sin(angle_1+angle_2),
                        -l2*np.sin(angle_1+angle_2)-h3*np.sin(angle_1+angle_2),
                        np.cos(angle_1+angle_2)]])
        # m=np.linalg.lstsq(np.dot(j.T,j),np.array([[1,0,0],[0,1,0],[0,0,1]]))[0]  #

        # pseudo_j = np.dot(m,j.T)
        """
        angle_1=q[0]
        angle_2=q[1]
        h3=0.012-q[2]


        j=np.array([[-l1*np.cos(angle_1)-l2*np.cos(angle_1+angle_2)-h3*np.cos(angle_1+angle_2),
                        -l2*np.cos(angle_1+angle_2)-h3*np.cos(angle_1+angle_2),
                        -np.sin(angle_1+angle_2)],

                    [l1*np.sin(angle_1)+l2*np.sin(angle_1+angle_2)+h3*np.sin(angle_1+angle_2),
                        l2*np.sin(angle_1+angle_2)+h3*np.sin(angle_1+angle_2),
                        -np.cos(angle_1+angle_2)]])        

        return j










class Serial3RKinematics():
    def __init__(self, 
            base_pivot=[0, 0, 0], 
            link_lengths=[0.3, 0.3, 0.3]):
        self.link_lengths = link_lengths
        self.base_pivot = base_pivot
        self.serial_2R = Serial2RKinematics([base_pivot[1], base_pivot[2]], [link_lengths[1], link_lengths[2]])

    def inverseKinematics(self, leg_ID, ee_pos, branch="<"):
        '''
        Inverse kinematics of a serial 3-R manipulator

        Note - Leg is in x-z plane, rotation about y

        Inputs:
        -- base_pivot: Position of the base pivot (in Cartesian co-ordinates)
        -- link_len: Link lenghts [l1, l2, l3]
        -- ee_pos: position of the end-effector [x, y, z] (Cartesian co-ordinates)
        -- branch: specifies the branch of the inverse kinematics solutions

        Output:
        -- Solutions to both the branches of the IK. Angles specified in radians.
        -- Note that the angle of the knee joint is relative in nature.
	    -- Note the hip is taken with respective to the negative z axis
        -- The solution can be in 2 forms, based on the branch selected 
        '''

        valid = True
        valid1 = True
        q = np.zeros(3, float)
        x_y_z_points = np.array(ee_pos) - np.array(self.base_pivot)
        [x, y, z] = x_y_z_points.tolist() 

        abd_link = self.link_lengths[0]
        l = math.sqrt(y**2 + z**2)
        if l < abd_link:
            valid = False
            return valid, q
        z_prime = -math.sqrt(l**2 - abd_link**2)
        t1 = math.atan2(-z_prime, abd_link)

        if leg_ID == "FR" or leg_ID == "BR":
            t2 = math.atan2(-y, -z)
            q[0] = PI/2 - t1 - t2
        else:
            t2 = math.atan2(y, -z)
            q[0] = t1 + t2 - PI/2

        x_prime = x

        valid1, [q[1], q[2]] = self.serial_2R.inverseKinematics([x_prime, z_prime], branch)

        if valid1 == False:
            #print("Point is outside the workspace")
            valid = False
            return valid, q

        return valid, q

    def forwardKinematics(self, leg_ID, q):
        '''
        Forward Kinematics of the serial 3-R manipulator

        Note - Leg is in x-z plane, rotation about y

        Args:
        --- q : A vector of the joint angles [q_abd, q_hip, q_knee], where q_knee is relative in nature
        Returns:
        --- v : The position vector of the end-effector
        '''
        rotX = lambda t : np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]]) 
        abd_link = self.link_lengths[0]
        v = np.zeros(3)

        q_abd = q[0]
        q_hip = q[1]
        q_knee = q[2]

        v_temp = self.serial_2R.forwardKinematics([q_hip, q_knee])
        
        if leg_ID == "FR" or leg_ID == "BR":
            v[1] = -abd_link
        else:
            v[1] = abd_link

        v[0] = v_temp[0]
        v[2] = v_temp[1]

        v = rotX(q_abd) @ v

        return v

class StochliteKinematics(object):
    '''
    Class to implement the position and velocity kinematics for the Stoch Lite leg
    Position kinematics: Forward kinematics, Inverse kinematics
    Velocity kinematics: Jacobian
    '''
    def __init__(self,
            base_pivot=[0, 0, 0],
            link_parameters=[0.096, 0.146 , 0.172]):
        self.base_pivot = base_pivot
        self.link_parameters = link_parameters
        self.serial_3R = Serial3RKinematics(base_pivot, link_parameters)

    def inverseKinematics(self, leg_ID, v, branch="<"):
        '''
        inverse kinematics  function
        Note - Leg is in x-z plane, rotation about positive y, positive x is 0 reference

        Args:
            v: Cartesian coordinate of end effector
            
            These conventions are based on right hand rule
        Ret:
            [motor_knee, motor_hip, motor_abduction] :  a list of knee, hip, and abduction motor angles to reach a (x, y, z) position
        '''

        abd_angle = 0
        hip_angle = 0
        knee_angle = 0        

        valid, q = self.serial_3R.inverseKinematics(leg_ID, v, branch)

        if valid:
            abd_angle = q[0]
            hip_angle = q[1]
            knee_angle = q[2]
        
        return valid,[abd_angle, hip_angle, knee_angle]

    def forwardKinematics(self, leg_ID, q):
        '''
        Forward kinematics of the Stoch Lite leg
        Args:
        -- q : A vector of the joint angles [q_abd, q_hip, q_knee], where q_knee is relative in nature
        Return:
        -- x : End-effector positions

        The conventions taken for this is the right hand rule which is x is forward y is left and z is up 
        '''

        v = self.serial_3R.forwardKinematics(leg_ID, q)

        return v


# Can be uncommented to test the ik of the robot
# make sure you are providing points in the workspace 
# of the robot


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
