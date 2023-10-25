from cProfile import label
import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt 
plt.style.use(['science','ieee'])
plt.rcParams['text.usetex'] = True
# data=pd.read_csv("base_robustness.txt",sep=',',lineterminator=']')
# data.to_csv("data1_robustness.csv")

# data_frame=pd.read_csv("data1_robustness.csv",lineterminator='[')
# data_frame.to_csv("data_new1_robustness.csv")

data_frame0=pd.read_csv("new_iter_combined.csv")
# data_frame1=pd.read_csv('new_iter1.csv')
# data_frame2=pd.read_csv("new_iter4.csv")
# print(data_frame2['Unnamed: 0'][0])
# data_frame3=pd.DataFrame(np.random.randn(90,4),columns=["kp_foot","Max-height","Range","Average-height"],index=list(range(0,90,1)))
# for i in range(90):
#     a=(data_frame2.iloc[i,2000:3999]).to_numpy()
#     b1=a[np.argmax(a)]+0.15
#     b2=a[np.argmin(a)]+0.15
#     b3=(b1+b2)/2.0
#     data_frame3['Max-height'][i]=b1 
#     data_frame3['Range'][i]=b1-b2
#     data_frame3['Average-height'][i]=b3 
#     data_frame3["kp_foot"][i]=3000+100*i  
# data_frame3.to_csv("new_iter_robustness.csv")
# plt.plot(data_frame0['kp_foot'],data_frame0['Reward'],color='red',label='6000kp')
# plt.plot(data_frame1["kp_foot"],data_frame1['Max-height'],color='blue',label='4700kp')
# d=data_frame2.to_numpy()
# e=data_frame0.to_numpy()
# frames=[data_frame2,data_frame0]
# result=pd.concat(frames)
# c=np.zeros(98)
# m=np.zeros(98)
# e=e.T 
# d=d.T 
# print(e)
# print(d)
# for i in range(98):
#     if i<8:
#         c[i]=d[0][i]
#         m[i]=d[1][i]
#     else :
#         k=i-8
#         c[i]=e[0][k]
#         m[i]=e[1][k]



plt.xlabel(r'$k_p$\text{ foot}')
plt.ylabel("Maximum height achieved in metres")
plt.plot(data_frame0["kp_foot"][:91],data_frame0["Max-height"][:91],label='6500kp')
plt.show()