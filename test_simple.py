from env.single_leg_env import SingleLegEnv
import time
import numpy as np
import matplotlib.pyplot as plt
import os
import faulthandler
faulthandler.enable()
# // bad code goes here
env = SingleLegEnv()
env.reset()




desired_ang_foot=[]
desired_ang_knee=[]
desired_ang_hip=[]
actual_position=[]
power_per_step=[]
reward=[]
base_pos=[]
tau=[]
tau2=[]
foot_pos=[]
imforce=[]
applied_tor=[]
ee_position_x=[]
ee_position_y=[]
for i in range(1):
    for _ in range(5600):
        # action=np.array([898.36, .90, 1200.44, 0.38, 1000,1])
        # action=np.array([800,0.0,800,0.0,2500,0])
        action=np.array([1472.2017874623132, 1.1329340897219196, 1061.6263074801868, 9.907756841754034, 592.4315795560001, 4.079290736397737])
        # action=np.array([8.22774703e+02, 1.75155578e+00, 1.32802939e+03, 3.82555581e+00,1.49853323e+04, 1.28242367e+01])
        r, dev, desired_angles_1,desired_angles_2,desired_angles_3, actual_pos, power, motor_position,torque,force,ee_pos_x,ee_pos_y=env.step(action)
        reward.append(r)
        desired_ang_foot.append(desired_angles_3)
        desired_ang_knee.append(desired_angles_2)
        desired_ang_hip.append(desired_angles_1)
        actual_position.append(actual_pos)
        power_per_step.append(power)
        base_pos.append(motor_position[0]+0.15)
        foot_pos.append(motor_position[3])
        imforce.append(force)
        applied_tor.append(torque)
        ee_position_x.append(ee_pos_x)
        ee_position_y.append(ee_pos_y)
        env.render()
# plt.plot(reward)
# plt.show()
# plt.plot(actual_pos[1])
# plt.show()
# plt.plot(desired_pos[1])
plt.plot(base_pos)
plt.grid()
plt.show()
# print(desired_ang)
# plt.plot(ee_position_x,ee_position_y)
plt.plot(desired_ang_foot)
# plt.plot(desired_ang_hip)
# plt.plot(desired_ang_knee)
# plt.plot(power_per_step)
plt.plot(foot_pos)
plt.show()
# plt.plot(imforce)
plt.plot(applied_tor)
plt.show()