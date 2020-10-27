#%%
import numpy as np
from numpy import linalg as la
import math as mt
import matplotlib.pyplot as plt

V1 = 200
V2 = 200
y10 = 100
y20 = 200
T1out = 16
T1in = 4
T1open = 12
T2out = 12
T2in = 16
T2open = 12

#Defining changes of concentration and placing them in matrix form
#y1dot = -16*(y1/V1) + 4*(y2/V2) + 12*0
#y1dot = -16*(y1/V1) + 4*(y2/V2) + 12*0
#Ydot = [[-0.08, 0.02], [0.08, -0.08]]*[[y1],[y2]]

#Eigenvalues
eigMat = np.array([[-0.08, 0.02], [0.08, -0.08]])
lambda1, congs = la.eig(eigMat)
print("The lambdas are: " + str(lambda1))
# Lambda 1 = 0.12
# Lambda 2 = -0.04

# Ydot = C1*[[1], [-2]]*e^-0.12t + C2*[[1], [2]]*e^-0.4t
# [[100], [200]] = [[C1+C2], [-2*C1+2*C2]] 
C1 = 0
C2 = 100

# y1t = 100*e^-0.04t
# y2t = 200*e^-0.04t

span = 100

y1Array = np.empty([span])
y2Array = np.empty([span])
time = np.empty([span])

for t in range(span):
    time[t] = t

def computeConc(tarray, constant):
    for t in range(span):
        tarray[t] = (constant*mt.pow(mt.e, (-0.04*t)))

def main():
    computeConc(y1Array, 100)
    computeConc(y2Array, 200)
    plt.plot(time, y1Array)
    plt.plot(time, y2Array)

main()

# %%
