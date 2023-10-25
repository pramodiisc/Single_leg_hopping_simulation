import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt 
plt.style.use(['science','ieee'])
plt.rcParams['text.usetex'] = True

# data=pd.read_csv("power2.txt",sep=',',lineterminator=']')
# data.to_csv("data_power_1.csv")

data_frame=pd.read_csv("data_power_1.csv")
# # data_frame.to_csv("data_new1_robustness.csv")
# # print(data_frame)
# # data_frame2=pd.read_csv("data_new1_robustness.csv")
# # print(data_frame2['Unnamed: 0'][0])
data_frame3=pd.DataFrame(np.random.randn(90,2),columns=["kp_foot","Average-power"],index=list(range(0,90,1)))
a=data_frame.to_numpy()
# a=a.T 
for i in range(90):
    
    # b1=a[np.argmax(a)]+0.15
    # b2=a[np.argmin(a)]+0.15
    # b3=(b1+b2)/2.0
    data_frame3['Average-power'][i]=data_frame.iloc[0,i] 
    # data_frame3['Range'][i]=b1-b2
    # data_frame3['Average-height'][i]=b3 
    data_frame3["kp_foot"][i]=1000+100*i  
data_frame3.to_csv("new_iter_power1.csv")
plt.ylabel("Average power consumed in Watt")
plt.xlabel(r'$k_p$\text{ foot}')
plt.plot(data_frame3['kp_foot'],data_frame3['Average-power'])
plt.show()