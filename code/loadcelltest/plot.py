import numpy as np
import matplotlib.pyplot as plt


data = np.loadtxt('loadcelldata.csv', delimiter=',')
data[:,0] = (data[:,0] - data[0,0]) / 1000.0

plt.plot(data[:,0], data[:,1])