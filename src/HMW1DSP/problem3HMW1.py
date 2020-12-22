#%%
import numpy as np
import math as mt
import matplotlib.pyplot as plt
import random

'''
Problem 2
A. Repeat part 1, with noisy signals; change the noise level between 0.01 and 0.2 of the signal
magnitude. Compare results as provided in the HW attached lecture notes.
B. Compare between 2-samples, and 3-samples methods in terms of accuracy for noisy signals.
C. Try to repeat the 2-samples, and 3-samples methods on a period time, and average the results,
would this provide any improvement?
'''

t_arr = np.array([[0.17, 0.182, 0.193], [0.014,0.016,0.018], [0.02,0.05,0.08],
[0.011,0.013,0.015], [0.005,0.01, 0.015], [0.01,0.015, 0.02]]) # time array of arbitrary size
f_arr = np.array([30, 50, 60, 120, 240, 360, 80]) # frequency array
Vm_arr = np.array([5, 110, 220, 380, 460, 620, 880]) # amplitude array
Qv_arr = np.array([mt.radians(0), mt.radians(60), mt.radians(20), # phase shift arrays
mt.radians(30), mt.radians(50), mt.radians(40)])
It = np.array([[0], [0], [0]])

number_of_samples = 3 #number of samples for the method to use
Version = 0 #version of parameters used

def printVariables(**args):
    for key, value in args.items():
        print(key , " = " , value)

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
        #plt.plot(time, voltage)
        #plt.figure()

    # get an arbitrary number of voltage samples
    def sampleVoltage(time):
        return (Vm * mt.sin(w*time + Qv) + Vm*random.uniform(-0.2, 0.2)) #noise

    def sampleVoltage2(time):
        return (Vm * mt.sin(w*time + Qv))

    def calculateQ(time):
        return w*time
    
    #createSine()

    # calculate n = [number_of_samples] Thetas
    Q = np.zeros(([number_of_samples]))
    for time in range(0, number_of_samples):
        Q[time] = calculateQ(t[time])

    # get n voltage samples
    Vtemp = np.array([-222, 212, 5])

    def analogToVoltage(analog):
        return np.interp(analog,[-512,512],[0,5])

    V = []
    for value in Vtemp:
        V.append(analogToVoltage(value))

    MySamples = np.zeros(3)
    for i in range(0,3):
        MySamples[i] = sampleVoltage2(t[i])

    print("Samples are \n", MySamples)
    

    # some linear algebra
    b = np.zeros((2,1))
    for k in range(0, number_of_samples):
        b[0][0] += V[k]*mt.sin(Q[k])
        b[1][0] += V[k]*mt.cos(Q[k])

    A = np.zeros((2,2))
    for k in range(0,number_of_samples):
        A[0][0] += mt.sin(Q[k])*mt.sin(Q[k])
        A[0][1] += mt.sin(Q[k])*mt.cos(Q[k])
        A[1][0] += mt.sin(Q[k])*mt.cos(Q[k])
        A[1][1] += mt.cos(Q[k])*mt.cos(Q[k])

    VmQv = np.dot(np.linalg.inv(A), b)

    # calculate phi 
    Qvv = mt.atan(VmQv[1]/VmQv[0])

    # calculate amplitude
    Vmv = VmQv[1]/mt.sin(Qvv)

    VmQv[0]

    # print everything
    printVariables(frequency = f, Amplitude = Vm, PhaseShift = mt.degrees(Qv),
     SampledVoltages = V, VmQv = VmQv, CalculatedPhaseShift = mt.degrees(Qvv), 
     CalculatedAmplitude = Vmv)

runSimulation(Version)

# %%
