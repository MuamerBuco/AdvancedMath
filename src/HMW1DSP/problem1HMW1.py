#%%
import numpy as np
import math as mt
import matplotlib.pyplot as plt

'''
Problem 1
A.	Generate a sinusoidal signal and use two samples method to estimate 
the phasor values of the signal. Use your method to at least six different signals in phase shift, 
amplitude and sampling frequency.

B.	Use a three-sample method for doing the same job in part A.

V(t) = Vm * sin(wt * Qv)
V(t) = 220 * sin(2*pi*50 + 30)

x = A^-1 * b
'''

t_arr = np.array([[0.012,0.018], [0.014,0.016], [0.02,0.05],
[0.011,0.013], [0.005,0.01], [0.01,0.03]]) # time array of arbitrary size
f_arr = np.array([50, 60, 120, 240, 360, 400]) # frequency array
Vm_arr = np.array([110, 220, 380, 460, 620, 880]) # amplitude array
Qv_arr = np.array([mt.radians(30), mt.radians(60), mt.radians(20), # phase shift arrays
mt.radians(30), mt.radians(30), mt.radians(30)])

def printVariables(**args):
    for key, value in args.items():
        print("Value " , key , " = " , value)

# first argument is omega(w), the rest is an arbitrary
# number of time instances
def calculateQ(*args):
    Q = []
    for time in args:
        Q.append(args[0]*time)
    Q.pop(0)
    return Q

def runSimulation(version):
    t = t_arr[version]
    f = f_arr[version]
    Vm = Vm_arr[version]
    Qv = Qv_arr[version]

    def createSine():
        time = np.linspace(0,0.1,1000)
        voltage = np.zeros([1000])
        for i in range(0, 1000):
            voltage[i] = Vm * mt.sin(w*time[i] + Qv)
        plt.plot(time, voltage)
        plt.figure()

    # get an arbitrary number of voltage samples
    def sampleVoltages(*args):
        V = []
        for Q in args:
            Vnext = (Vm*mt.sin(Q)*mt.cos(Qv) + Vm*mt.sin(Qv)*mt.cos(Q))
            V.append(Vm*mt.sin(Q)*mt.cos(Qv) + Vm*mt.sin(Qv)*mt.cos(Q))
        return V
    
    w = 2*mt.pi*f

    createSine()

    # calculate theta1 and theta2
    Q = calculateQ(w, t[0], t[1])

    # get 2 voltage samples
    V = sampleVoltages(Q[0], Q[1])

    # some linear algebra
    Q12 = np.array([[mt.sin(Q[0]), mt.cos(Q[0])], 
                    [mt.sin(Q[1]), mt.cos(Q[1])]])

    VmQv = np.dot(np.linalg.inv(Q12), V)

    # calculate phi 
    Qvv = mt.atan(VmQv[1]/VmQv[0])

    # calculate amplitude
    Vmv = VmQv[1]/mt.sin(Qvv)

    # print everything
    printVariables(Q = Q, V = V, VmQv = VmQv, Qvv = mt.degrees(Qvv), Vmv = Vmv)

runSimulation(0)
# %%
