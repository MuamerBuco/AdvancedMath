#%%
import numpy as np
import matplotlib.pyplot as plt
num1 = 30
r = np.arange(num1+10)
p = np.arange(num1)
R,P = np.meshgrid(r,p)
data = np.random.random((num1,num1))
data2 = np.zeros((num1,num1))
#plt.pcolor(R,P,data2)
#plt.scatter(R[:-1,:-1]+0.5,P[:-1,:-1]+0.5, color = 'white')
plt.scatter(R,P)
#spacex = np.linspace(0, 1000, 0.1)
#spacey = np.linspace(0, 1000, 0.1)
