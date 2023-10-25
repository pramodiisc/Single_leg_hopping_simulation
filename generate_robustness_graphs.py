from pyrsistent import b
from env.single_leg_env import SingleLegEnv
import time
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree
import os
plt.style.use(['science','ieee'])
plt.rcParams['text.usetex'] = True
desired_position=[]
actual_position=[]
pos3=[]
kp=[]
power_per_step=[]
actual_position=[]
# env.reset()
reward=float(0.00)
d=[]
base_pos=[]
reward=[]
reward_final=[]
graph_1=[]
base_1=[]
base_2=[]
action=np.zeros((4,4))
# action[0] = np.array([896.989,1.0855,901.0025,1.009]) #trained for some kp foot
# action[1]=np.array([900.5387397315353, 0.8672765335999428, 899.2592474631965, 0.04289948392691929]) #trained for 4700 kp
# action[2]=np.array([902.321096521594, 3.682804175523155, 900.1792983606956, 0.6640177814799676]) #trained for 6000 kp
action[0]=np.array([898.3681347292417, 3.905550632744719, 899.4493696767773, 0.38320494171132363]) #trained for 6500 kp
power_per_step1=[]
iteration=[]
for i in range(100):
    number=1000+100*i
    # number=3000
    s=str(number)
    print(s)
    os.chdir('./env/assets/')
    et = xml.etree.ElementTree.parse("single_leg.xml")

    new_tag = et.getroot()

    new_tag[5][3].attrib['kp']=s
    
    et.write("single_leg.xml")

    os.chdir('../../')
    kp.append(1000+100*i)
    env = SingleLegEnv()
    env.reset()
    reward=[]
    base_pos=[]
    knee_t=[]
    hip_t=[]
    # fig = plt.figure()
    # ax = plt.axes(projection ='3d')
    
    # action[0]+=np.random.randint(-100,100)
    # action[1]+=np.random.randint(-1,1)/10.00
    # action[2]+=np.random.randint(-100,100)
    # action[3]+=np.random.randint(-1,1)/10.00
    # iteration=[]
    power1=0.00
    for j in range(4000):
        # action = np.array([100,5,20,25,40,5])
        # action = np.array([1270.50589169  , 65.45971518 , 699.48515787 ,  59.0399418 ])
        # action = np.array([1.33694044e+03 ,2.55026456e+01 ,1.12520337e+03 ,1.54716880e-01])#6000 foot kp
        # action = np.array([1193.77772931,   23.61489952, 1255.77683383 ,   1.58034005])#8000 foot kp
        # 10000 foot_kpo

        r, dev, desired_pos, actual_pos, power, base_position, torque=env.step(action[0])
        reward.append(r)
        d.append(dev)
        hip_t.append(torque[1])
        knee_t.append(torque[2])
        desired_position.append(desired_pos)
        actual_position.append(actual_pos)
        power1+=(power)
        # base_pos.append(base_position)
        # iteration.append(j)
        # env.render()
    power1/=4000.00
    r1=reward[np.argmax(reward)]
    reward_final.append(r1)
    # f=open("base_robustness.txt","a")
    # f.write(str(base_pos))
    # f.close()
    # b1=base_pos[2000+np.argmax(base_pos[2000:4000])]
    # # base_1.append(base_pos[2000:4000])
    # # base_2.append(base_pos[2000:4000])
    # b2=base_pos[2000+np.argmin(base_pos[2000:4000])]
    # graph=b1
    # plt.plot(iteration[90:4000],power_per_step[90:4000])
    # plt.show()
    power_per_step1.append(power1)
    iteration.append(i)
    # iteration1[1]

    # print(b1)
    # print(b2)
f=open("power2.txt","a")
f.write(str(power_per_step1))
f.close()
plt.xlabel(r'$k_p$\text{ foot}')
plt.ylabel("Average power consumed in Watt")
plt.plot(kp,power_per_step1)
plt.show()
plt.xlabel("kp foot")
plt.ylabel("Average power consumed")
plt.scatter(kp,power_per_step1,color='red')
plt.show()
# plt.plot(iteration,power_per_step1[0],color='blue')
# plt.plot(iteration,power_per_step1[1],color='green')
# plt.plot(iteration,power_per_step1[2],color='red')
# plt.plot(iteration,power_per_step1[3],color='yellow')
# plt.show()


    # f=open("base_robustness.txt","a")
    # f.write(str(graph))
    # f.close()

    # print(graph)
    # graph_1.append(graph)

    # print("reward",r1)
# plt.plot(base_pos[2000:4000])
# plt.show()
# plt.plot(base_2)
# plt.show()
# print(r1) 
# base_pos1=np.asarray(base_pos)
# plt.plot(kp,graph_1)
# plt.show()
# plt.scatter(kp,graph_1)
# plt.show()
# plt.plot(kp,reward_final)
# plt.show()


# plt.plot(d)
# plt.show()

# x=[p[0] for p in desired_position]
# z=[p[1] for p in desired_position]
# plt.plot(x,z,'blue')

# x1=[p[0] for p in pos3]
# z1=[p[1] for p in pos3]
# plt.plot(x1,z1,'red')

# x2=[p[0] for p in actual_position]
# z2=[p[1] for p in actual_position]
# plt.plot(x2,z2,'green')

# plt.show()

# plt.plot(base_pos)
# plt.show()

# plt.plot(power_per_step)
# plt.show()


#pos.append(reward)

# hip_ang  = [p[0] for p in desired_angle]
# knee_ang = [p[1] for p in desired_angle]
# foot_pos = [p[2] for p in desired_angle]

# plt.plot  (hip_ang ,    label = "hip_ang ")
# plt.plot  (knee_ang,    label = "knee_ang")
# plt.plot  (foot_pos,    label = "foot_pos")

# plt.legend()
# plt.show()

# plt.plot(r)
# plt.show()