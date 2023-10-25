from env.single_leg_env import SingleLegEnv
import genetic
import time
import numpy as np
import matplotlib.pyplot as plt
from ga import geneticalgorithm as ga

#policy = np.load("experiments/13Oct1 (copy)/iterations/best_policy.npy")
env = SingleLegEnv()
# pos=[]
# kp=[]
# pos3=[]
# actual_position=[]
f=open("ga_library_1.txt","w")
env.reset()
'''
Every evolutionary algorithm (metaheuristic) has some parameters to be adjusted. 
Genetic algorithm also has some parameters.
The parameters of geneticalgorithm is defined as a dictionary:

'''
algorithm_param = {'max_num_iteration': 50,\
                   'population_size':150,\
                   'mutation_probability':0.1,\
                   'elit_ratio': 0.01,\
                   'crossover_probability': 0.5,\
                   'parents_portion': 0.3,\
                   'crossover_type':'uniform',\
                   'max_iteration_without_improv':None}

def get_reward(action):
    reward=float(0.00)
    # reward=[]
    env.reset()
    for i in range(400):        
        r,dev,pos1,pos2,pos3,power_perstep=env.step(action)
        # reward.append(r) #anubhab
        reward+=r
    # r1=np.argmax(reward)        
    # return -1*r1 #anubhab
    return reward

fun=get_reward 
varbound=np.array([[200,2000],[0,100],[200,2000],[0,100]])
vartype=np.array([['real'],['real'],['real'],['real']])
'''
function fun so that its output is the objective function we want to minimize
dimension  set of decision variables
varbound boundaries for variables must be defined as a numpy array and for each variable we need a separate boundary
'''

model=ga(function=fun,\
            dimension=4,\
            variable_type_mixed=vartype,\
            variable_boundaries=varbound,\
            algorithm_parameters=algorithm_param)
model.run()
'''
access to the best answer of the defined optimization problem found by geneticalgorithm as a 
dictionary and a report of the progress of the genetic algorithm
'''
convergence=model.report
solution=model.ouput_dict
# print("solution",solution)
# fun=get_reward 
# x0=np.array([900,0.1,900,1])
# sigma0=1
# x,es=cma.fmin2(fun,x0,sigma0)
# print(es.result)
f.write("Best gain="+str(solution)+"\n")
f.write(solution)
f.close()




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
'''
1. frequency also we can learn
2. Trajectory has to be refined or checked
3. Foot kp, kd by putting motor wac check
4. foot and motor damping can be tweeked i.e in motors
5. Trajectory variation

'''