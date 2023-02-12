import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

var = pd.read_excel("traj-cdf.xlsx")
print(var)

#To plot the trajectory

"""xl = list(var['XLEAD'])
yl = list(var['YLEAD'])

x1 = list(var['X1'])
y1 = list(var['Y1'])

x2 = list(var['X2'])
y2 = list(var['Y2'])

plt.plot(xl,yl,'g', label = 'TrajLead')

plt.plot(x1,y1,'r', label = 'TrajFol1')

plt.plot(x2,y2,'b', label = 'TrajFol2')"""

#To plot the speed

dlead = np.sort(list(var['DLEAD']))

dfol1 = np.sort(list(var['DFOL1']))

dfol2 = np.sort(list(var['DFOL2']))

ydlead = 1. * np.arange(len(list(var['DLEAD']))) / (len(list(var['DLEAD'])) - 1)

ydfol1 = 1. * np.arange(len(list(var['DFOL1']))) / (len(list(var['DFOL1'])) - 1)

ydfol2 = 1. * np.arange(len(list(var['DFOL2']))) / (len(list(var['DFOL1'])) - 1)


plt.xlabel('distance from the correct trajectory')
plt.ylabel('CDF')

plt.plot(dlead, ydlead, linestyle='solid', color='g', label = 'Leader')

plt.plot(dfol1, ydfol1, linestyle='dashed', color='r', label = 'Follower1')

plt.plot(dfol2, ydfol2, linestyle='dotted', color='b', label = 'Follower2')

plt.legend()

plt.show()