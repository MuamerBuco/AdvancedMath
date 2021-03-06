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

t_arr = np.array([[0.012,0.018,0.024], [0.014,0.016,0.018], [0.02,0.05,0.08],
[0.011,0.013,0.015], [0.005,0.01, 0.015], [0.01,0.015, 0.02]]) # time array of arbitrary size
f_arr = np.array([50, 60, 120, 240, 360, 80]) # frequency array
Vm_arr = np.array([110, 220, 380, 460, 620, 880]) # amplitude array
Qv_arr = np.array([mt.radians(30), mt.radians(60), mt.radians(20), # phase shift arrays
mt.radians(30), mt.radians(50), mt.radians(40)])

number_of_samples = 2 #number of samples for the method to use
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
        plt.plot(time, voltage)
        plt.figure()

    # get an arbitrary number of voltage samples
    def sampleVoltage(time):
        return (Vm * mt.sin(w*time + Qv) + Vm*random.uniform(-0.2, 0.2))

    def calculateQ(time):
        return w*time
    
    createSine()

    # calculate n = [number_of_samples] Thetas
    Q = np.zeros(([number_of_samples]))
    for time in range(0, number_of_samples):
        Q[time] = calculateQ(t[time])

    # get n voltage samples
    V = np.zeros(([number_of_samples]))
    for time in range(0, number_of_samples):
        V[time] = sampleVoltage(t[time])

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

    Verror = abs(Vm - Vmv)
    Qerror = abs(Qv - Qvv)

    # print everything
    printVariables(frequency = f, Amplitude = Vm, PhaseShift = mt.degrees(Qv), Qs = Q,
    SampledVoltages = V, VmQv = VmQv, CalculatedPhaseShift = mt.degrees(Qvv), 
    CalculatedAmplitude = Vmv)

    return Verror, mt.degrees(Qerror)

num_measurements = 3
results_array = np.empty([num_measurements,2])
for i in range(0,num_measurements):
    results_array[i] = runSimulation(Version)

avg_error = np.mean(results_array, axis = 0)
print("The average error for [Voltage, Phase] =", avg_error, "for ", num_measurements, "measurements")
