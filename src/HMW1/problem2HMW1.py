#%%
import math as mt
import numpy as np
import matplotlib.pyplot as plt

### ODE Equations
# di^2/dt^2 + R/L*(di/dt) + 1/LC*i = 0
# It = A*pow(math.e, st)
# di/dt = A*s*pow(math.e, st)
# di^2/dt^2 = A*s^2*pow(math.e, st)

### Characteristic Equation
# A*pow(math.e, st)*(s^2 + R/L*s + 1/(L*C)) = 0

### Substitution: s^2 + R/L*s + 1/L*C = 0

### Critically Damped
# Alpha = R/2*L
# Wo = 1/sqrt(L*C)
# Alpha^2 = Wo^2
# di^2/dt^2 + 2*Alpha*(di/dt) + Alpha^2*i = 0
# u = di/dt + Alpha*i
# du/dt = -A1*Alpha*pow(math.e, -Alpha*t)

#R = 4, L = 0.5, C = 0.125, A1 = 0, A2 = 20
# It = (A1 + A2*t)*mt.pow(mt.e, -4*t)
# It = 20*t*mt.pow(mt.e, -4*t)

span = 200
aArray = np.empty(span)
bArray = np.empty(span)
time = np.linspace(0,5,span)

for i in range(span):
    t = i/20
    tarray[i] = 20*t*mt.pow(mt.e, -4*t)

### Under-damped
# (R/2*L)^2 < 1/L*C
# R = 0.5, L = 0.5, C = 0.125
# It = e^t-/2 * (0*(cos(sqrt(63)/2) + 5.04*sin(sqrt(63)/2)*t)
# It = 5.04*e^-t/2*sin(sqrt(63)/2)*t

for i in range(span):
    t = i/20
    tarray[i] = 5.04* mt.pow(mt.e, (-t/2)) *mt.sin(3.968*t)

plt.plot(time, aArray)
plt.plot(time, bArray)

# %%
