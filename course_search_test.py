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
    with open('search_oct4.csv','w') as file:
        write = writer(file,delimiter=',',lineterminator='\n')
        write.writerow(header)
        file.close()

    # final_result = dict()
    # final_result['i'] = list(range(600, 1200, 100))
    # final_result['j'] = list(np.arange(0, 1.5, 0.5))
    # final_result['k'] = list(range(600, 1200, 100))
    # final_result['l'] = list(np.arange(0, 1.5, 0.5))
    # final_result['m'] = list(range(1000, 10000, 500))
    # final_result['n'] = list(range(0, 8, 2))

    # {i: list(range(100, 25500,100)), j: list(np.arange(0,5,0.5))}
    r1_list=[]
    
    for i in range(600,1200,100):
        for j in range(0,3,1):
            for k in range(600,1200, 100):
                for l in range(0,3,1): #225
                    for m in range(900,5000, 500): #2500  #15*225 3375
                        for n in range(0,6,2):
                            reward=[]
                            for _ in range(4700):
                                
                                j1=0.01+(j*0.5)
                                l1=0.01+(l*0.5)
                                n1=0.01+(n)
                                action=np.array([i,j1,k,l1,m,n1])
                                # r, dev, desired_angles_1,desired_angles_2,desired_angles_3, actual_pos, power, motor_position,torque,force,ee_pos_x,ee_pos_y=env.step(action)
                                ht,force,power=env.step(action)
                                reward.append(ht)
                            
                            r1=reward[2299+np.argmax(reward[2300:4700])]
                            # r1=reward[np.argmax(reward)]
                            # env.reset()
                            with open('search_oct4.csv','a') as file:
                                write_object = writer(file)
                                write_object.writerow([i,j1,k,l1,m,n1,r1])
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
