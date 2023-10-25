import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt 
from scipy import stats
plt.style.use(['science','ieee'])
plt.rcParams['text.usetex'] = True
# data=pd.read_csv("base.txt",sep=',',lineterminator=']')
# data.to_csv("data1.csv")

# data_frame=pd.read_csv("data1.csv",lineterminator='[')
# data_frame.to_csv("data_new1.csv")

data_frame2=pd.read_csv("data_new6.csv")
# print(data_frame2['Unnamed: 0'][0])
# data_frame3=pd.DataFrame(np.random.randn(90,4),columns=["kp_foot","Max-height","Range","Average-height"],index=list(range(0,90,1)))

a=(data_frame2.iloc[12,0:3999]).to_numpy() + 0.15
x=np.linspace(0,3998,3999)
mean=np.zeros(3999)
std_old=np.zeros(3999)

mean[0]=a[0]
for i in range(3998):
    k=i+1
    mean[k]=(mean[i]*k + a[k])/(k+1)
    std_old[k]=np.std(mean[0:(k+1)])


# plt.scatter(x,mean,color='red')
# plt.plot(x,a[0:2000],color='blue')

plt.xlabel("No. of simulation steps")
plt.ylabel("Cumulative mean height in metres")
plt.plot(x,mean)
# plt.grid()
plt.show()

plt.xlabel("No. of simulation steps")
plt.ylabel("Standard deviation of the base-position")
plt.plot(x,std_old)
plt.show()
mean_new=[]
std=[]
down=[]
up=[]
x1=np.linspace(1000,10000,100)
for i in range(100):
    a=(data_frame2.iloc[i,2000:3999]).to_numpy() + 0.15
    error_new=stats.sem(a)
    mean_new.append(np.mean(a))
    std.append(np.std(a))
    mean=np.zeros(1999)
    std_old=np.zeros(1999)

    mean[0]=a[0]
    for i in range(1998):
        k=i+1
        mean[k]=(mean[i]*k + a[k])/(k+1)
        std_old[k]=np.std(mean[0:(k+1)])
    

    b=np.mean(a-mean)
    down.append(error_new)
    # up.append(mean[np.argmax(mean)])

plt.xlabel(r'$k_p$\text{ foot}')
plt.ylabel("Mean height in metres with SEM")
plt.errorbar(x1,mean_new,down)
# plt.scatter(x1,down)
# plt.plot(mean_new)
plt.show()

plt.xlabel(r'$k_p$\text{ foot}')
plt.ylabel("Standard error of mean")
plt.plot(x1,down)
plt.show()

# b1=(np.sum(a))/2000.00 + 0.15
# b2=a[np.argmin(a)]+0.15
# b3=(b1+b2)/2.0
# data_frame3['Max-height'][i]=b1 
# data_frame3['Range'][i]=b1-b2
# data_frame3['Average-height'][i]=b3 
# data_frame3["kp_foot"][i]=3000+100*i  
# data_frame3.to_csv("new_iter.csv")
# plt.plot(data_frame3['kp_foot'],data_frame3['Average-height'])
# plt.show()