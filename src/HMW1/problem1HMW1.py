#%%
import math
import matplotlib.pyplot as plt
'''
L/y1 = 1+b*pow(math.e, -t*K)
(L/y1 - 1)/b = pow(math.e, -t*K)
'''

#Calculate K and b params using P(t=0),P(t=1 and max population
# KBparams[2] = { b, K}
def GetKandB(L, y1, y0, t2): #assume y1 = Y(0)
    b = (L/y0) - 1 
    K = -1*math.log(((L/y1 - 1)/b)/t2)
    KBparams = [K, b]
    return KBparams

def iterate(K, b, L, time, TYarray):
    Yt = L/(1 + b*pow(math.e ,K*time))
    time += 1
    print("Day " + str(time) + ": " + str(Yt) + '\n')
    if(Yt < L - 1):
        TYarray[0].append(time)
        TYarray[1].append(Yt)
        iterate(K, b, L, time, TYarray)
         
#Run iterations to reach max pop
def SolvePopulation(t2, p1, p2, maxPop):
    KandB = GetKandB(maxPop, p1, p2, t2)
    K = KandB[0]
    b = KandB[1]
    TYarray = [ [0, t2], [p1, p2] ]
    iterate(K, b, maxPop, t2, TYarray)
    print("K = " + str(K) + " ,b = " + str(b) + 'n')
    print("Solution: Y(t) = maxPop/(1 + " + str(b) + "*pow(math.e ," + str(K) + "*t))")
    plt.plot(TYarray[0], TYarray[1])

#Final analytical solution is: N(t)=45000/1+149*e^-1.22*t
def main():
    t2 = 1.00
    p1 = 300
    p2 = 1000
    maxPop = 45000
    
    SolvePopulation(t2, p1, p2, maxPop)
    
main()
# %%
