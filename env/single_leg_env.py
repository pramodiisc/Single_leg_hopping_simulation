from re import A
import numpy as np
from gym import utils
from gym import spaces
import mujoco_py
import env.mujoco_env as mujoco_env
from utils.ik_class import Serial_RRP_Kinematics
import numpy as np
import math as m
import os
import xml.etree.ElementTree
PI = np.pi
import matplotlib.pyplot as plt
import matplotlib



DEFAULT_CAMERA_CONFIG = {
    'distance': 1.0,
}


class SingleLegEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self,
                 xml_file='single_leg.xml',
                 forward_reward_weight=1.0,
                 ctrl_cost_weight=0.1):
        utils.EzPickle.__init__(**locals())
        self.pos=[]
        self.total_power= 0        
        self.link_lengths = [0.172 , 0.125]

        self.total_power = 0     

        # self.leg_step_length = 0.024
        self.leg_step_length = 0

        
        # self.z_foot = -0.175 # for base position of 20cm
        # self.z_foot=0.14597174 # Base position 35cm
        
        # for 5 cm desired jump
        self.z_foot = -0.174
        self.step_height = 0.026

        # # for 10 cm desired jump
        # self.z_foot = -0.15
        # self.step_height = 0.05

        self.motor_torque=np.array([0.00,0.00,0.00,0.00],dtype=np.float32)
        #self.dt = 0.008
        self.clips=7
        self.frequency = 4.0
        
        self.omega = 2 * np.pi * self.frequency

        self.time_step = 0
        self.time_step_max = 300
        # self.phase = 0
        self.phase =self.omega*0.0001*1 #to keep trajectorty point one step ahead
        self._forward_reward_weight = forward_reward_weight

        self._ctrl_cost_weight = ctrl_cost_weight


        self.action_space = spaces.Box(
            low=-np.ones(6),
            high=np.ones(6),
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(8,),
            dtype=np.float32,
        )

        self.all_phases = np.linspace(0, 2*np.pi, num=200)

        self.all_desired_pos = []

        # for phase in self.all_phases:
        #     desired_pos, _ = self.calc_desired_posvel(phase)
        #     self.all_desired_pos.append(desired_pos)


        self.all_desired_pos = np.array(self.all_desired_pos)

        self.serial_2r = Serial_RRP_Kinematics()

        
        mujoco_env.MujocoEnv.__init__(self, xml_file, 2)

    def control_cost(self, action):
        control_cost = self._ctrl_cost_weight * np.sum(np.square(action))
        return control_cost
    
  
    def step(self, action):
        

        # number1=action[4]
        # number2=action[5]
        # s1=str(number1)
        # s2=str(number2)

        # print(s1,s2)
        # os.chdir('./assets/')
        # et = xml.etree.ElementTree.parse("single_leg.xml")

        # new_tag1 = et.getroot()
        # new_tag2 = et.getroot()

        # new_tag1[5][3].attrib['kp']=s1
        # new_tag2[7][1][2][3][4][4][4][1].attrib['damping']=s2
        
        # et.write("single_leg.xml")
        # os.chdir('../')


        '''
        Manual constraint on the sliding foot link.
        '''
            
        if self.sim.data.qpos[10]<0.0 :
            self.sim.data.qpos[10]=0
            self.set_state(self.sim.data.qpos,self.sim.data.qvel)
        if self.sim.data.qpos[10]>0.012 :
            self.sim.data.qpos[10]=0.012
            self.set_state(self.sim.data.qpos,self.sim.data.qvel)


        current_motor_position = self.GetMotorAngles()
        current_motor_velocity = self.GetMotorVelocities()
        self.phase = self.phase + self.omega*self.dt  

        def constrain_phase(theta):           
            theta = np.fmod(theta, 2 * PI)
            # print("nppi", theta)
            if (theta < 0):

                theta = theta + 2 * PI
            return theta

        self.phase=constrain_phase(self.phase)
        self.desired_pos, self.desired_vel = self.calc_desired_posvel(self.phase)        
        _,self.desired_angles= self.serial_2r.inverseKinematics(self.phase,branch='<')

        # if self.desired_angles[2]>0.010:
        #     self.desired_angles[2]=0.010
        # if self.desired_angles[2]<0:
        #     self.desired_angles[2]=0

        # print("self.desired_angles",self.desired_angles)

        self.pos=self.serial_2r.forwardKinematics(self.desired_angles)
        self.desired_angvel = np.dot(np.linalg.pinv(self.serial_2r.Jacobian(self.desired_angles)),self.desired_vel)
        b=np.array([0,current_motor_position[0]])
        desired_ee_pos, desired_ee_vel = self.calc_desired_posvel()
        q=np.array([current_motor_position[1],current_motor_position[2],current_motor_position[3]])
        forward_pos=self.serial_2r.forward_kin_actual(q,b)

        kp_hip  = action[0]
        kd_hip  = action[1]
        kp_knee = action[2]
        kd_knee = action[3]
        kp_foot = action[4]
        kd_foot = action[5]

        force=self.contact_force()
        print("contact_force",force)
      
        self.motor_torque[0]=0.00
        self.motor_torque[1] = kp_hip *(self.desired_angles[0] - current_motor_position[1]) + kd_hip*(self.desired_angvel[0] - current_motor_velocity[1])
        self.motor_torque[2] = kp_knee*(self.desired_angles[1] - current_motor_position[2]) + kd_knee*(self.desired_angvel[1]- current_motor_velocity[2])
        self.motor_torque[3] = (kp_foot*( -current_motor_position[3]) + kd_foot*( current_motor_velocity[3]))
        # self.motor_torque[3]= (self.desired_angles[2]-current_motor_position[3])
        # print("motor_torque",self.motor_torque)
        applied_motor_torque=self.motor_torque
        applied_motor_torque[0:3]= np.clip(np.array(self.motor_torque[0:3]),-self.clips,self.clips) # self.clips=5
        # print("applied_motor_torque_knee",applied_motor_torque)
        # applied_motor_torque=[*applied_motor_torque,*self.motor_torque[3] ]
         
        # print("applied_motor_torque_foot",applied_motor_torque)
        self.do_simulation(applied_motor_torque, self.frame_skip)
        new_mot_pos = self.GetMotorAngles()
        new_mot_vel = self.GetMotorVelocities()
        new_obs=np.array([*new_mot_pos,*new_mot_vel])
        self.power_perstep = self.power_consumed(self.motor_torque[1:3], current_motor_velocity[1:3])
        
        reward,dev_traj = self.calc_reward()
        # pint()
        reward=new_mot_pos[0]
        reward_ht=new_mot_pos[0]+0.15
        reward_force= force
        reward_power=self.power_perstep
        
        # return observation, reward, terminal, info
        # return reward,dev_traj,self.desired_angles[0],self.desired_angles[1],self.desired_angles[2],forward_pos, self.power_perstep,new_mot_pos,applied_motor_torque[3], force,desired_ee_pos[0],desired_ee_pos[1] #for test file
        # return reward_ht,reward_force,reward_power
        return reward_ht,reward_force,reward_power,self.desired_pos, new_mot_pos,reward_power,current_motor_position[0]
    def calc_desired_posvel(self, phase=None):
        '''
        Return desired pos and desired velocity
        leg_step_length: Major axis
        z_foot         : Center for ellipse
        step_height    : Minor axis
        '''
        current_motor_position = self.GetMotorAngles()
        current_motor_velocity = self.GetMotorVelocities()
        if phase is None:
            phase = self.phase

        
        # x =   -0.0216+self.leg_step_length * np.cos(phase)
        # x =  -0.03
        x =  -0.03
        z = self.z_foot - self.step_height * np.cos(phase)
        xdot = -self.leg_step_length * np.sin(phase) * self.omega
        zdot = self.step_height * np.sin(phase) * self.omega
        # else:
        #     x = -0.096 + self.leg_step_length * np.cos(2*np.pi - phase)
        #     z = self.z_foot + self.step_height * np.sin(2*np.pi - phase)
        #     xdot = -self.leg_step_length * np.sin(phase) * self.omega
        #     zdot = self.step_height * np.cos(2*np.pi - phase) * self.omega

        # if phase > np.pi:
        #     x = 0.00594727-0.1+self.leg_step_length * np.cos(phase)
        #     z = self.z_foot+ self.step_height * np.sin(phase)
        #     #z=self.z_foot + self.step_height * np.sin(2*np.pi - phase)
        #     xdot = -self.leg_step_length * np.sin(phase) * self.omega
        #     zdot = self.step_height * np.cos(2*np.pi - phase) * self.omega
        # else:
        #     x = 0.00594727-0.1+self.leg_step_length * np.cos(phase)
        #     z = self.z_foot+self.step_height * np.sin(phase)
        #     xdot = -self.leg_step_length * np.sin(phase) * self.omega
        #     zdot = self.step_height * np.cos(2*np.pi - phase) * self.omega

        return np.array([x, z]), np.array([xdot, zdot])

    def power_consumed(self, motor_torque, motor_vel):
        '''
        Calculates total power consumed (sum of all motor powers) as a multiplication of torque and velocity
        '''
        total_power = 0
        for torque, vel in zip(motor_torque, motor_vel):
            power = torque * vel
            total_power=abs(power)
            # total_power += abs(power)
        return total_power

    # def clc_contact_force(self):
    
    #     geom2_body=0
    #     force=0
    #     # while self.sim.data.ncon =1:
    #     for i in range(self.sim.data.ncon):
    #         # global geom2_body
    #         # Note that the contact array has more than `ncon` entries,
    #         # so be careful to only read the valid entries.
    #         contact = self.sim.data.contact[i]

    #         # print('contact', i)
    #         # print('dist', contact.dist)
    #         # print('geom1', contact.geom1, self.sim.model.geom_id2name(contact.geom1))
    #         # print('geom2', contact.geom2, self.sim.model.geom_id2name(contact.geom2))

    #         # There's more stuff in the data structure
    #         # See the mujoco documentation for more info!
    #         geom2_body = self.sim.model.geom_bodyid[self.sim.data.contact[i].geom2]
    #         # print(' Contact force on geom2 body', self.sim.data.cfrc_ext[geom2_body])
    #         # print('norm', np.sqrt(np.sum(np.square(self.sim.data.cfrc_ext[geom2_body]))))
    #         force=np.sqrt(np.sum(np.square(self.sim.data.cfrc_ext[geom2_body])))
    #         # Use internal functions to read out mj_contactForce
    #         c_array = np.zeros(6, dtype=np.float64)
    #         print('c_array', c_array)
    #         mujoco_py.functions.mj_contactForce(self.sim.model, self.sim.data, i, c_array)
    #         print('c_array', c_array)
    #     return force,geom2_body

    def contact_force(self):

        contact = self.sim.data.contact[0]
        geom2_body = self.sim.model.geom_bodyid[self.sim.data.contact[0].geom2]
        c_array = np.zeros(6, dtype=np.float64)
        mujoco_py.functions.mj_contactForce(self.sim.model, self.sim.data,0, c_array)
        force=np.sqrt(np.sum(np.square(c_array[0:3])))

        return force

    def calc_reward(self):
        '''
        calculate reward based on tracking error
        reward = np.exp(np.linalg.norm(current_ee_pos - desired_ee_pos))
        '''
        '''
        calc is state is terminal
        1. if jerk very large?
        2. if tracking error is more than threshold
        return true or false for terminal
        '''
        all_motor_position=self.GetMotorAngles()
        all_motor_velocity=self.GetMotorVelocities()
        current_motor_pos=all_motor_position[1:4]
        current_cartesian_pos = self.serial_2r.forward_kin_actual(current_motor_pos,np.array([0,all_motor_position[0]]))
        current_motor_vel= all_motor_velocity[1:4]

        current_ee_pos = current_cartesian_pos  #ee - end effector
        # current_ee_vel = np.dot(np.linalg.pinv(self.serial_2r.Jacobian(current_cartesian_pos)), current_motor_vel)
        current_ee_vel = np.dot(self.serial_2r.Jacobian(current_motor_pos), current_motor_vel)  
        desired_ee_pos, desired_ee_vel = self.calc_desired_posvel()  # should be a function of the phase

        deviation = np.linalg.norm(current_ee_pos - desired_ee_pos)
        dev_traj=current_ee_pos-desired_ee_pos
        deviation_vel = np.linalg.norm(current_ee_vel - desired_ee_vel)

        
        reward = np.exp(-1*(0.05-all_motor_position[0])) #Anubhab
        terminal = (deviation > 0.2) or (abs(current_ee_pos[0]) > 0.2) \
            or (current_ee_pos[1] > -0.15) or (self.time_step >= self.time_step_max)

        return reward,dev_traj #, terminal, {'pos_reward': deviation, 'vel_reward': deviation_vel}


    def GetMotorAngles(self):
        '''
        This function returns the current joint angles in order [FLH FLK FRH FRK BLH BLK BRH BRK FLA FRA BLA BRA ]
        '''
        motor_ang = self.sim.data.qpos.flat.copy()[7:]
        return motor_ang

    def GetMotorVelocities(self):
        '''
        This function returns the current joint velocities in order [FLH FLK FRH FRK BLH BLK BRH BRK FLA FRA BLA BRA ]
        '''        
        motor_vel = self.sim.data.qvel.flat.copy()[6:]
        return motor_vel

    def GetBasePosAndOrientation(self):
        '''
        This function returns the robot torso position(X,Y,Z) and orientation(Quaternions) in world frame
        '''
        position=self.sim.data.qpos.flat.copy()[:3]
        orientation=self.sim.data.qpos.flat.copy()[3:7]
        return position,orientation

    def GetBaseAngularVelocity(self):
        '''
        This function returns the robot base angular velocity in world frame
        Ret: list of 3 floats
        '''
        basevelocity= self.sim.data.qvel.flat.copy()[2:5]
        #print('basevelocity', basevelocity)
        return basevelocity

    def GetBaseLinearVelocity(self):
        '''
        This function returns the robot base linear velocity in world frame
        Ret: list of 3 floats
        '''
        basevelocity= self.sim.data.qvel.flat.copy()[:2]
        return basevelocity


    def reset_model(self):

        qpos = self.init_qpos
        qpos[8] = 1.2
        qpos[9] = -2.05
        # #qpos[8]=0.5
        # #qpos[9]=-1
        qpos[10]=0.0
        qpos[7]=-0.15             
        qvel = self.init_qvel 
        self.set_state(qpos, qvel)
        new_mot_pos = self.sim.data.qpos[7:]
        new_mot_vel = self.sim.data.qvel[6:] 

        new_obs=np.array([*new_mot_pos,*new_mot_vel])
        observation = new_obs

        return observation

    def viewer_setup(self):
        for key, value in DEFAULT_CAMERA_CONFIG.items():
            if isinstance(value, np.ndarray):
                getattr(self.viewer.cam, key)[:] = value
            else:
                setattr(self.viewer.cam, key, value)
        self.viewer._paused=True

"""
1. actions will be all kp kd.
2. reward will contain all the errors while following the trajectory.
3. remove target height from reward.
4. Add trajectory in the envirmnment file
5. Calculate x,z position for known initial position
6. Calculate trajectory accordingly
7. make last link as position actuator
8. limit the link length and put if condition for <0 and >0.012m
9. make one more 2R manipulator class with link length as hip and knee only
10. put control part away and compare RRP and forward kin graphs
11. 
"""

# if __name__ == '__main__':
#     env=SingleLegEnv()
#     env.reset()
#     print(env.sim.data.ncon)