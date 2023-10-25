# soumya code
from matplotlib import pyplot as plt
import numpy as np
import math
from scipy import optimize

PI = np.pi


class Serial_RRP_Kinematics():

    def __init__(self, 
            base_pivot=[0,0], 
            link_lengths=[0.172,0.158]):
        self.link_lengths = link_lengths
        self.base_pivot = np.array([0,-0.15])
        self.delta_dt=0.01

        self.dt=0.0005*2
        self.frequency = 4
        self.omega = 2 * np.pi * self.frequency
        self.d_phase=self.dt*self.omega
        self.f=[]
        # self.f.append((1.8945,   -3.7713 ,   0.0078))
        self.f.append((1.2*np.pi, 0, 0.006))

        self.p=20

    def traj_gen(self,phase):
        z=-0.174-0.025*np.sin(phase)
        x=0
        return x,z

    def inverseKinematics(self, phase, branch="<"):

        valid = True
        [l1, l2] = self.link_lengths

        x0=self.f[len(self.f)-1]
        
        m,n=self.traj_gen(phase)
        print("mn",m,n)
        
        ans=optimize.fmin_slsqp(self.objective,x0,eqcons=[self.eqcon1,self.eqcon2],bounds=[(-2*np.pi,2*np.pi),(-2*np.pi,2*np.pi),(0.0,0.012)],args=(m,n))
        
        
        self.f.append(ans)
        valid = True

        return valid,ans,m,n
        return valid, np.array([self.f[len(self.f)-1][0],self.f[len(self.f)-1][1],0.054-self.f[len(self.f)-1][2]]),m,n


    def objective(self,a,m,n):

        return (a[0]**2 + a[1]**2 + (self.p*a[2])**2)

    def eqcon1(self,a,m,n):
        [l1,l2]=self.link_lengths
        return l1*np.cos(1.5*np.pi-a[0])+l2*np.cos(1.5*np.pi-a[0]+a[1])+(self.p*a[2])*np.cos(1.5*np.pi-a[0]+a[1])-m

    def eqcon2(self,a,m,n):
        [l1,l2]=self.link_lengths
        return l1*np.sin(1.5*np.pi-a[0])+l2*np.sin(1.5*np.pi-a[0]+a[1])+(self.p*a[2])*np.sin(1.5*np.pi-a[0]+a[1])-n
    
    def forward(self,a):
        [l1,l2]=self.link_lengths
        x= l1*np.cos(1.5*np.pi-a[0])+l2*np.cos(1.5*np.pi-a[0]+a[1])+(self.p*a[2])*np.cos(1.5*np.pi-a[0]+a[1])
        z =l1*np.sin(1.5*np.pi-a[0])+l2*np.sin(1.5*np.pi-a[0]+a[1])+(self.p*a[2])*np.sin(1.5*np.pi-a[0]+a[1])
        return x,z
# if __name__ == '__main__':
#     #s = Serial2RKin([0,0],[0.15,0.175])
#     thata_1=[]
#     thata_2=[]
#     thata_3=[]
#     phase=0
#     omega = 2 * np.pi *4
#     dt=0.0005
#     s = Serial_RRP_Kinematics()
    
#     for i in range(1000):
#         phase = phase + omega*dt  
#         print("phase",phase)
#         a=s.constrain_phase(0)
#         print("a",a)
#         phase=s.constrain_phase(phase)

#         _,theta=s.inverseKinematics(phase,branch='<')

#     thata_1.append(theta[0])
#     thata_2.append(theta[1])
#     thata_3.append(theta[2])

#     plt.plot(thata_1)
#     plt.plot(thata_2)
#     plt.plot(thata_3)
#     plt.show()


if __name__ == '__main__':
    #s = Serial2RKin([0,0],[0.15,0.175])
    thata_1=[]
    thata_2=[]
    thata_3=[]
    x=[]
    z=[]
    phase=0
    omega = 2 * np.pi *4
    dt=0.0005
    s = Serial_RRP_Kinematics()
    e=[]
    f=[]
    
    for i in np.arange(0.0,50.0,0.1):
        _,theta,m,n=s.inverseKinematics(i,branch='<')
        c,d=s.forward(theta)
        thata_1.append(theta[0])
        thata_2.append(theta[1])
        thata_3.append(theta[2])
        x.append(m)
        z.append(n)
        e.append(c)
        f.append(d)
thetaa=[1.3,1.9976,0.005] #(2.0869085666092024e-06, -0.24373741374375504)
pos_init=s.forward(thetaa)
print("pos_init",pos_init)

plt.plot(thata_1)
plt.show()
plt.plot(thata_2)
plt.show()
plt.plot(thata_3)
plt.show()
# plt.plot(x)
# plt.plot(z)
plt.plot(e)
plt.plot(f)
plt.show()