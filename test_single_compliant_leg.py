import sys, os
import csv
import env.single_leg_env as e
import argparse
# from fabulous.color import blue,green,red,bold
import math as m
import gym
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use( 'tkagg' )

PI = np.pi

with open("torque", 'w', newline='') as f:
	thewriter =csv.writer(f)
	
	if (__name__ == "__main__"):
		parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
		
		parser.add_argument('--PolicyDir', help='directory of the policy to be tested', type=str, default='15Jun3')
		parser.add_argument('--MotorStrength', help='maximum motor Strength to be applied', type=float, default=7.0)
		parser.add_argument('--seed', help='seed for the random sampling', type=float, default=1000)
		parser.add_argument('--EpisodeLength', help='number of gait steps of a episode', type=int, default=5000)

		args = parser.parse_args()


		env = e.SingleLegEnv()
		obs = env.reset()

		plot_data = []      
		act_mot_ang= []
		theta_mot =[]
		torque =[]
		raw_pow=[]
		reward_pow=[]
		raw_height=[]

		for i in range(30):
			a = 0.1
			b = 0.025
		
			
			for j in range(361,0, -2):
				# Ellipse
				x = a * m.cos(m.radians(j))
				x=0
				# z = 2*0.025+ b * m.sin(m.radians(j))
				z= -0.175+ b * m.sin(m.radians(j)) # x=0.1
				path=[x,z]
				_ , theta=env.inverseKinematics(path ,branch="<" )
				action = np.insert(np.append(theta,[1200, 5]),0,0)
				# action=([0,0,0,0,0])
				# print("action_test",action)
				m_vel_cmd_ext = np.zeros(4)
				mot_torque=env._apply_pd_control(action,m_vel_cmd_ext,action)
				actual_mot_angle =env.GetMotorAngles()

				raw_reward, pow, height,tot= env._get_reward()
				# print("power",pow,"total_power",tot, "ht reward",height)
				env.step(action)
				# print("get base pos",env.sim.data.qpos.flat.copy())
				env.render()

				plot_data.append(path)
				act_mot_ang.append(actual_mot_angle)
				theta_mot.append(action)
				torque.append(mot_torque)
				raw_pow.append(tot)
				reward_pow.append(pow)
				raw_height.append(height)

				# print ('render')
				# print("outer",i)
				# thewriter.writerow(motor_torque) # to write in csv file

		x = [p[0] for p in plot_data]
		z = [p[1] for p in plot_data]
		# linera = [p[2] for p in plot_data]

		mot_1 = [m[0] for m in act_mot_ang]
		mot_2 = [m[1] for m in act_mot_ang]
		mot_3 = [m[2] for m in act_mot_ang]
		mot_4 = [m[3] for m in act_mot_ang]

		theta_mot_1 = [t[0] for t in theta_mot]
		theta_mot_2 = [t[1] for t in theta_mot]
		theta_mot_3 = [t[2] for t in theta_mot]
		theta_mot_4 = [t[3] for t in theta_mot]


		torque_1 = [n[0] for n in torque]
		torque_2 = [n[1] for n in torque]
		torque_3 = [n[2] for n in torque]
		torque_4 = [n[3] for n in torque]
		
		plt.plot  (x,      label = "path_x")
		plt.plot  (z,     label = "path_z")
		# plt.plot  (linera,     label = "path_linera")    
		plt.legend()
		plt.show()

		plt.plot  (mot_1,      label = "slot_actual_mot_1")
		plt.plot  (theta_mot_1,      label = "slot_commanded_theta_mot_1")

		plt.legend()
		plt.show()

		plt.plot  (mot_2,     label = "hip_actual_mot_2")
		plt.plot  (theta_mot_2,     label = "hip_commanded_theta_mot_2")

		plt.legend()
		plt.show()

		plt.plot  (mot_3,     label = "knee_actual_mot_3")
		plt.plot  (theta_mot_3,     label = "knee_commanded_theta_mot_3")

		plt.legend()
		plt.show()
		
		plt.plot  (mot_4,     label = "spring_actual_mot_4")
		# plt.plot  (theta_mot_4,     label = "spring_commanded_theta_mot_4")


		plt.legend()
		plt.show()

		plt.plot  (torque_1,     label = "slot_torque_1")
		plt.plot  (torque_2,     label = "hip_torque_2")
		plt.plot  (torque_3,     label = "knee_torque_3")
		plt.plot  (torque_4,     label = "spring_torque_4")
		plt.legend()
		plt.show()


		plt.plot  (reward_pow,      label = "Reward_pow")
		plt.legend()
		plt.show()     

		plt.plot  (raw_pow,      label = "raw_pow")


		plt.legend()
		plt.show()

		plt.plot (x,z )
		plt.legend()
		plt.show()
