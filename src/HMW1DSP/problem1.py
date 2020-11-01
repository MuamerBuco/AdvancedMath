import numpy as np
import math as mt
import matplotlib.pyplot as plt

'''
Problem 1
A.	Generate a sinusoidal signal and use two samples method to estimate the phasor values of the signal. Use your method to at least six different signals in phase shift, 
amplitude and sampling frequency.
B.	Use a three-sample method for doing the same job in part A.

V(t) = Vm * sin(wt * Qv)
V(t) = 220 * sin(2*pi*50 + 30)
'''

t0 = 0
t1 = 0.0005
t2 = 0.001

def sampleV(time, Vm, Qv):
    return Vm * mt.sin(2*mt.pi*50*time + Qv)

print (sampleV(0.00125, 220, 30))

time = np.linspace(0,1000,5)

voltage = np.empty(1000)
for x in range(0,1000):
    voltage[x] = sampleV(time[x], 220, 30)

plt.plot(time, voltage)

