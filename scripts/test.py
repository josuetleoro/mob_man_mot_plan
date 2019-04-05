import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 10, 100)

fig, (ax1,ax2) = plt.subplots(2,1)
ax1.plot(x, np.sin(x), label='sin(x)')
ax1.set(ylabel='sin(x)')
#ax1.set_title("Simple Plot")
ax1.legend()

ax2.plot(x, np.cos(x), label='cos(x)')
ax2.set(xlabel='time', ylabel='cos(x)')
#ax2.set_title("Simple Plot")
ax2.legend()

plt.show()