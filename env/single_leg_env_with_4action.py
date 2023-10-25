from re import A
import numpy as np
from gym import utils
from gym import spaces
import env.mujoco_env as mujoco_env
# from utils.ik_class import Serial2RKinematics
import numpy as np
import math as m
PI = np.pi
import matplotlib.pyplot as plt
import matplotlib



DEFAULT_CAMERA_CONFIG = {
    'distance': 4.0,
}


class SingleLegEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self,
                 xml_file='single_leg.xml',
                 forward_reward_weight=1.0,
                 ctrl_cost_weight=0.1,
                 reset_noise_scale=0.1):
        utils.EzPickle.__init__(**locals())


        # self._kp = 400
        # self._kd = 2


        # self._kp_spring =100
        # self._kd_spring =20

        # self._kp_base = 0
        # self._kd_base = 0
        self.total_power= 0        
        self.link_lengths = [0.146 , 0.172]

        self.total_power = 0     

        self._forward_reward_weight = forward_reward_weight

        self._ctrl_cost_weight = ctrl_cost_weight

        self._reset_noise_scale = reset_noise_scale

        self.action_space = spaces.Box(
            low=-np.ones(4),
            high=np.ones(4),
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(8,),
            dtype=np.float32,
        )

        self.target_height = 0.10

        mujoco_env.MujocoEnv.__init__(self, xml_file, 2)

    def control_cost(self, action):
        control_cost = self._ctrl_cost_weight * np.sum(np.square(action))
        return control_cost

    def step(self, action):

        curr_height = self.sim.data.qpos[7]
        curr_z_velocity = self.sim.data.qvel[6]
        
        curr_hip_joint_pos = self.sim.data.qpos[8]
        curr_hip_joint_vel = self.sim.data.qvel[7]

        curr_knee_joint_pos = self.sim.data.qpos[9]
        curr_knee_joint_vel = self.sim.data.qvel[8]

        curr_foot_joint_pos = self.sim.data.qpos[10]
        curr_foot_joint_vel = self.sim.data.qvel[9]

        hip_tau = 50*action[0]
        knee_tau = 50*action[1]
        foot_kp = 50*action[2]
        foot_kd = 50*action[3]

        foot_tau = foot_kp * (0 - curr_foot_joint_pos) + foot_kd * (0 - curr_foot_joint_vel)

        motor_torque = np.array([hip_tau, knee_tau, foot_tau])

        self.do_simulation(motor_torque, self.frame_skip)

        new_height = self.sim.data.qpos[7]
        new_z_velocity = self.sim.data.qvel[6]
        print("height", new_height)        
        new_hip_joint_pos = self.sim.data.qpos[8]
        new_hip_joint_vel = self.sim.data.qvel[7]

        new_knee_joint_pos = self.sim.data.qpos[9]
        # print("knee_joint", new_knee_joint_pos)
        new_knee_joint_vel = self.sim.data.qvel[8]

        new_foot_joint_pos = self.sim.data.qpos[10]
        new_foot_joint_vel = self.sim.data.qvel[9]



        self.total_power = self.power_consumed (motor_torque[:2], self.sim.data.qvel[7:9])
        # print(self.total_power)

        new_obs = np.array([new_height, new_z_velocity, new_hip_joint_pos, new_knee_joint_pos, new_foot_joint_pos, new_hip_joint_vel, new_knee_joint_vel, new_foot_joint_vel])

        observation = new_obs
        reward = np.exp(-(self.target_height - new_height)**2) + np.exp(-new_z_velocity**2)-np.exp(-self.total_power/500)
        done = False
        info = {}

        return observation, reward, done, info

    def power_consumed(self, motor_torque, motor_vel):
        '''
        Calculates total power consumed (sum of all motor powers) as a multiplication of torque and velocity
        '''
        total_power = 0
        for torque, vel in zip(motor_torque, motor_vel):
            power = torque * vel
            total_power += abs(power)
        return total_power

    def reset_model(self):

        qpos = self.init_qpos
        qpos[8] = 1.09377797 
        qpos[9] = -1.94802971

        qpos[7]=0
             
        qvel = self.init_qvel 

        self.set_state(qpos, qvel)

        new_height = self.sim.data.qpos[7]
        new_z_velocity = self.sim.data.qvel[6]
        
        new_hip_joint_pos = self.sim.data.qpos[8]
        new_hip_joint_vel = self.sim.data.qvel[7]

        new_knee_joint_pos = self.sim.data.qpos[9]
        new_knee_joint_vel = self.sim.data.qvel[8]

        new_foot_joint_pos = self.sim.data.qpos[10]
        new_foot_joint_vel = self.sim.data.qvel[9]
        
        new_obs = np.array([new_height, new_z_velocity, new_hip_joint_pos, new_knee_joint_pos, new_foot_joint_pos, new_hip_joint_vel, new_knee_joint_vel, new_foot_joint_vel])

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


"""

