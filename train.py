from env.single_leg_env import SingleLegEnv
import genetic
import time
import numpy as np
import matplotlib.pyplot as plt
import cma
import pandas as pd
from scipy.signal import find_peaks
#policy = np.load("experiments/13Oct1 (copy)/iterations/best_policy.npy")
def run():
    env = SingleLegEnv()
    # pos=[]
    # kp=[]
    # pos3=[]
    # actual_position=[]
    
    env.reset()
    def get_reward(action):
        f=open('policy_log.txt',"a")
        reward=[]
        env.reset()
        for i in range(400):        
            r, desired_pos, actual_pos, power, base_position=env.step(action)
            reward.append(r)
            f.write(str(base_position))
            f.write('\n')
        r1=reward[np.argmax(reward)]
        f.close()
        return -r1 #anubhab

    fun=get_reward 


    x0=np.array([900,0.1,900,1,10000,2])
    '''
    sigma-standard deviation

    '''
    sigma0=1
    
    x,es=cma.fmin2(fun,x0,sigma0,{'bounds': [([150,0,150,0,500,0]),([1400,4,1400,4,100000,30])]})
    print(es.result)
    




    # initial_population=genetic.pop_gen()
    # pop=initial_population
    # reward_graph=[]
    # for i in range(20):
    #     gains=genetic.get_gains(pop)
    #     rewards=[]
    #     for j in range(len(gains)):
    #         action=np.array([gains[j][0],gains[j][1],gains[j][2],gains[j][3]])
    #         reward=0.0
    #         for _ in range(4000):
    #             r,pos1,pos2,pos3=env.step(action)
    #             reward+=r
    #         env.reset()
    #         print(reward)
            
    #         rewards.append(reward)
    #     old_pop=pop
    #     pop,max_reward=genetic.select_evolve(old_pop,rewards)
    #     reward_graph.append(max_reward)
    #     f.write("Best reward="+str(max_reward)+"\n")
    #     final_pop=genetic.best_indiv(old_pop,rewards)
    #     gain=genetic.get_gains(final_pop)
    #     f.write("Best gain="+str(gain)+"\n")
    # f.close()

    # final_pop=genetic.best_indiv(old_pop,rewards)
    # gain=genetic.get_gains(final_pop)

    # print(gain)
    # action=np.array([gain[0][0],gain[0][1],gain[0][2],gain[0][3]])
    # reward=0.0
    # for _ in range(4000):
    #     r,pos1,pos2,pos3=env.step(action)
    #     reward+=r
    # print(reward)

    # plt.plot(reward_graph)
    # plt.show()





    # reward=float(0.00)
    # for i in range(1):
    #     kp=200
    #     kd=2
    #     action = np.array([kp,kd,kp,kd])
    #     #action[2]= 45+0.05*i
    #     #action[3]=action[2]/80.00 
    #     # kp.append(action[2])
    #     #obs = env.reset()
    #     #print(obs)
    #     #env.render()
    #     #env.reset()
    #     # reward=0.00
    
    #     for _ in range(400):
    #         # action = policy.dot(obs)
    #         # print("policy", policy) 
    #         #action = np.array([100,5,20,25,40,5])
    #         # pos1=(env.step(action))
    #         pos1,pos2,des_pos=env.step(action)
    #         pos.append(pos1)
    #         pos3.append(pos2)
    #         actual_position.append(des_pos)
    #         env.render()
        
    #     #pos.append(reward)

    # # hip_ang  = [p[0] for p in desired_angle]
    # # knee_ang = [p[1] for p in desired_angle]
    # # foot_pos = [p[2] for p in desired_angle]

    # # plt.plot  (hip_ang ,    label = "hip_ang ")
    # # plt.plot  (knee_ang,    label = "knee_ang")
    # # plt.plot  (foot_pos,    label = "foot_pos")

    # # plt.legend()
    # # plt.show()

    # x=[p[0] for p in pos]
    # y=[0 for p in pos]
    # z=[p[1] for p in pos]
    # # ax=plt.axes(projection='3d')
    # # ax.plot3D(x,y,z,'red')
    # plt.plot(x,z,'blue')
    # # plt.show()

    # x2=[p[0] for p in actual_position]
    # y2=[0 for p in actual_position]
    # z2=[p[1] for p in actual_position]
    # plt.plot(x2,z2,'green')

    # x1=[p[0] for p in pos3]
    # y1=[0 for p in pos3]
    # z1=[p[1] for p in pos3]
    # # ax=plt.axes(projection='3d')
    # # ax.plot3D(x1,y1,z1,'blue')
    # plt.plot(x1,z1,'red')
    # #plt.plot(kp,pos)

    # plt.show()
