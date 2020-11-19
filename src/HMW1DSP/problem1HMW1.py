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
x = A^-1 * b
'''

t_arr = np.array([[0.012,0.018], [0.014,0.016], [0.02,0.05],
[0.011,0.013], [0.005,0.01], [0.01,0.015]]) # time array of arbitrary size
f_arr = np.array([50, 60, 120, 240, 360, 80]) # frequency array
Vm_arr = np.array([110, 220, 380, 460, 620, 880]) # amplitude array
Qv_arr = np.array([mt.radians(30), mt.radians(60), mt.radians(20), # phase shift arrays
mt.radians(30), mt.radians(50), mt.radians(40)])

def printVariables(**args):
    for key, value in args.items():
        print(key , " = " , value)

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

    w = 2*mt.pi*f

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
        for t in args:
            V.append(Vm * mt.sin(w*t + Qv))
        return V
    
    createSine()

    # calculate theta1 and theta2
    Q = calculateQ(w, t[0], t[1])

    # get 2 voltage samples
    V = sampleVoltages(t[0], t[1])

    # some linear algebra
    Q12 = np.array([[mt.sin(Q[0]), mt.cos(Q[0])], 
                    [mt.sin(Q[1]), mt.cos(Q[1])]])

    VmQv = np.dot(np.linalg.inv(Q12), V)

    # calculate phi 
    Qvv = mt.atan(VmQv[1]/VmQv[0])

    # calculate amplitude
    Vmv = VmQv[1]/mt.sin(Qvv)

    # print everything
    printVariables(frequency = f, Amplitude = Vm, PhaseShift = mt.degrees(Qv), Qs = Q,
     SampledVoltages = V, VmQv = VmQv, CalculatedPhaseShift = mt.degrees(Qvv), 
     CalculatedAmplitude = Vmv)

runSimulation(5)
# %%
