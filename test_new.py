from pyrsistent import b
from env.single_leg_env import SingleLegEnv
import time
import numpy as np
import matplotlib.pyplot as plt
# plt.style.use(['science','ieee'])
import xml.etree.ElementTree
import os

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
hip_angle=[]
knee_angle=[]
spring_length=[]
for i in range(1):
    # number=900+250*i
    number=1000
    s=str(number)
    print(s)
    # os.chdir('./env/assets/')
    # et = xml.etree.ElementTree.parse("single_leg.xml")

    # new_tag = et.getroot()

    # new_tag[5][3].attrib['kp']=s
    
    # et.write("single_leg.xml")

    # os.chdir('../../')
    kp.append(s)
    env = SingleLegEnv()
    env.reset()
    reward=[]
    base_pos=[]
    tau=[]
    tau2=[]
    for _ in range(4000):
        # action = np.array([100,5,20,25,40,5])
        # action = np.array([1270.50589169  , 65.45971518 , 699.48515787 ,  59.0399418 ])
        # action = np.array([1.33694044e+03 ,2.55026456e+01 ,1.12520337e+03 ,1.54716880e-01])#6000 foot kp
        # action = np.array([1193.77772931,   23.61489952, 1255.77683383 ,   1.58034005])#8000 foot kp
        # action = np.array([896.989,1.0855,901.0025,1.009])# 10000 foot_kpo
        # action=np.array([900.5387397315353, 0.8672765335999428, 899.2592474631965, 0.04289948392691929])
        action=np.array([898.3681347292417, 3.905550632744719, 899.4493696767773, 0.38320494171132363])

        r, dev, desired_pos, actual_pos, power, base_position, torque=env.step(action)

        reward.append(r)
        tau.append(torque[1])
        tau2.append(torque[2])
        
        d.append(dev)
        desired_position.append(desired_pos)
        actual_position.append(actual_pos)
        power_per_step.append(power)
        base_pos.append(base_position[0]+0.15)
        hip_angle.append(base_position[1])
        knee_angle.append(base_position[2])
        spring_length.append(base_position[3]+0.37)
        env.render()
    r1=reward[np.argmax(reward)]
    reward_final.append(r1)
    hip_max=np.mean(tau)
    knee_max=np.mean(tau2)
    # print(hip_max)
    # print(knee_max)
    # f1=open("reward.txt","a")
    # f1.write(str(reward_final))
    # f1.close()
    # f=open("base.txt","a")
    # f.write(str(base_pos))
    # f.close()
    b1=base_pos[2000+np.argmax(base_pos[2000:4000])]
    # base_1.append(base_pos[2000:4000])
    # base_2.append(base_pos[2000:4000])
    b2=base_pos[2000+np.argmin(base_pos[2000:4000])]
    graph=b1-b2
    # print(b1)
    # print(b2)
    
    # f=open("base.txt","a")
    # f.write(str(graph))
    # f.close()

    # print(graph)
    graph_1.append(graph)

    # print("reward",r1)
# plt.xlabel("Hip and knee Angle")
# plt.ylabel("Hip and knee motor torque")
# plt.scatter(hip_angle[3000:3200],tau[3000:3200],color='blue')
# plt.show()
# plt.scatter(knee_angle[3000:3200],tau2[3000:3200],color='red')
# plt.show()

# plt.scatter(kp,base_pos[2000+np.argmax(base_pos[2000:4000])])
# plt.show()
# plt.xlabel("No. of simulation steps")
# plt.ylabel("Hip angle in radians")
# plt.plot(hip_angle)
# plt.show()

# plt.xlabel("No. of simulation steps")
# plt.ylabel("Base position in metres")
# plt.plot(base_pos)
# plt.show()
# plt.xlabel("No. of iterations")
# plt.ylabel("Knee Angle")

# plt.plot(knee_angle)
# plt.show()
# plt.xlabel("No. of iterations")
# plt.ylabel("Spring length")
# plt.plot(spring_length)
# plt.show()
# plt.plot(base_2)
# plt.show()
# print(r1) 
# base_pos1=np.asarray(base_pos)
# plt.plot(tau)
# plt.show()
# plt.plot(tau2)
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