import pandas as pd
import matplotlib.pyplot as plt
var = pd.read_excel("traj.xlsx")
print(var)

# To plot the trajectory
xl = list(var['XLEAD'])
yl = list(var['YLEAD'])

x1 = list(var['XF1'])
y1 = list(var['YF1'])

x2 = list(var['XF2'])
y2 = list(var['YF2'])

plt.plot(xl,yl,'g',label='TrajLead')
plt.plot(x1,y1,'r',label='TrajFol1')
plt.plot(x2,y2,'b',label='TrajFol2')

#To plot the speed
"""slead = list(var['SLEAD'])

sfol1 = list(var['SFOL1'])

sfol2 = list(var['SFOL2'])

plt.plot(range(len(slead)), slead, 'g', label = 'SpeedLead')
plt.plot(range(len(sfol1)), sfol1, 'r', label = 'SpeedFol1')
plt.plot(range(len(sfol2)), sfol2, 'b', label = 'SpeedFol2')"""

plt.legend()
plt.show()
