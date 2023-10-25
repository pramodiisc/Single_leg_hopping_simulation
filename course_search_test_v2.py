from env.single_leg_env import SingleLegEnv
import time
import numpy as np
import matplotlib.pyplot as plt
import os
import faulthandler
import csv
from csv import writer

faulthandler.enable()
# // bad code goes here
env = SingleLegEnv()
env.reset()
for i in range(1):

    # desired_ang_foot=[]
    # desired_ang_knee=[]
    # desired_ang_hip=[]
    # actual_position=[]
    # power_per_step=[]
    
    # base_pos=[]
    # tau=[]
    # tau2=[]
    # foot_pos=[]
    # imforce=[]
    # applied_tor=[]
    # ee_position_x=[]
    # ee_position_y=[]
    # env.render()
    # f=open('policy_log.txt',"a")
    header = ['kp_hip','kd_hip','kp_knee','kd_knee','kp_foot','kd_foot','ht']
    with open('search.csv','w') as file:
        write = writer(file,delimiter=',',lineterminator='\n')
        write.writerow(header)
        file.close()

    final_result = dict()
    final_result['i'] = list(range(700, 1200, 100))
    final_result['j'] = list(np.arange(0, 1.5, 0.5))
    final_result['k'] = list(range(700, 1200, 100))
    final_result['l'] = list(np.arange(0, 1.5, 0.5))
    final_result['m'] = list(range(1500, 10000, 500))
    final_result['n'] = list(range(0, 8, 2))

    # {i: list(range(100, 25500,100)), j: list(np.arange(0,5,0.5))}
    r1_list=[]
    for i in range(700,1200,100):
        for j in np.arange(0,1.5,0.5):
            for k in range(700,1200, 100):
                for l in np.arange(0,1.5,0.5):
                    for m in range(500,3000, 500):
                        for n in range(0,8,2):
                            
                            height=[]
                            reward=[]
                            for o in range(10): #to store and compare last two height
                           
                                for _ in range(1200):
                                    action=np.array([i,j,k,l,m,n])
                                    # r, dev, desired_angles_1,desired_angles_2,desired_angles_3, actual_pos, power, motor_position,torque,force,ee_pos_x,ee_pos_y=env.step(action)
                                    ht,force,power=env.step(action)
                                    reward.append(ht)
                                
                                r1=reward[np.argmax(reward)]
                                height[0]=0
                                height.append(r1)
                                
                                for i in len(height):
                                    
                                    if abs(height[i]-height[i-1])<10^-3:
                                        r2=height[i]
                                        break           

                            with open('search.csv','a') as file:
                                write_object = writer(file)
                                write_object.writerow([i,j,k,l,m,n,r2])
                                file.close()
                            env.reset()




    # final_result['r1']=r1_list

                            # writer.writerow(i,j,k,l,m,n,r1)
                            
                            # desired_ang_foot.append(desired_angles_3)
                                # desired_ang_knee.append(desired_angles_2)
                                # desired_ang_hip.append(desired_angles_1)
                                # actual_position.append(actual_pos)
                                # power_per_step.append(power)
                                # base_pos.append(motor_position[0]+0.15)
                                # foot_pos.append(motor_position[3])
                                # imforce.append(force)
                                # applied_tor.append(torque)
                                # ee_position_x.append(ee_pos_x)
                                # ee_position_y.append(ee_pos_y)
                            # env.render()
# plt.plot(reward)
# plt.show()
# plt.plot(actual_pos[1])
# plt.show()
# plt.plot(desired_pos[1])
# plt.plot(base_pos)
# print(desired_ang)
# plt.plot(ee_position_x,ee_position_y)
# plt.plot(desired_ang_foot)
# # plt.plot(desired_ang_hip)
# # plt.plot(desired_ang_knee)
# # plt.plot(power_per_step)
# plt.plot(foot_pos)
# plt.show()
# # plt.plot(imforce)
# plt.plot(applied_tor)
# plt.show()
