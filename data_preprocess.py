import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt 

data=pd.read_csv("base1.txt",sep=',',lineterminator=']')
data.to_csv("data6.csv")

data_frame=pd.read_csv("data6.csv",lineterminator='[')
data_frame.to_csv("data_new6.csv")

data_frame2=pd.read_csv("data_new6.csv")
# print(data_frame2['Unnamed: 0'][0])
data_frame3=pd.DataFrame(np.random.randn(190,4),columns=["kp_foot","Max-height","Range","Average-height"],index=list(range(0,190,1)))
for i in range(190):
    a=(data_frame2.iloc[i,2000:3999]).to_numpy()
    b1=a[np.argmax(a)]+0.15
    b2=a[np.argmin(a)]+0.15
    b3=(b1+b2)/2.0
    data_frame3['Max-height'][i]=b1 
    data_frame3['Range'][i]=b1-b2
    data_frame3['Average-height'][i]=b3 
    data_frame3["kp_foot"][i]=1000+100*i  
data_frame3.to_csv("new_iter_combined.csv")
plt.plot(data_frame3['kp_foot'],data_frame3['Max-height'])
plt.show()