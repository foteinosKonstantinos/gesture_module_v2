import matplotlib.pyplot as plt
import math

# https://matplotlib.org/stable/gallery/lines_bars_and_markers/marker_reference.html

# orientation w.r.t. to the magnetic north (+90 deg) and clockwise

trajectory = {'detection position (xy)': [(328.5544554455517, 1642.998858683394), (1328.554455445569, 2642.998858683386), (2328.5544554455937, 3642.998858683378), (3328.554455445627, 4642.998858683367), (4328.554455445668, 5642.998858683357), (5328.554455445718, 6642.998858683346), (6328.5544554457765, 7642.998858683333), (7328.554455445839, 8642.99885868332)], 'robot position (xy)': [(0.0, 0.0), (999.9999999999999, 999.9999999999999), (1999.9999999999998, 1999.9999999999998), (3000.0, 3000.0), (3999.9999999999995, 3999.9999999999995), (5000.0, 5000.0), (6000.0, 6000.0), (7000.0, 7000.0)], 'robot orientation (deg)': [360.0, 360.0, 360.0, 360.0, 360.0, 360.0, 360.0, 360.0]}


x_r = [xy[0]/1000 for xy in trajectory['robot position (xy)']]
y_r = [xy[1]/1000 for xy in trajectory['robot position (xy)']]

x_d = [xy[0]/1000 for xy in trajectory['detection position (xy)']]
y_d = [xy[1]/1000 for xy in trajectory['detection position (xy)']]

# radians
omega = [(-omega+90) #/ 180 * math.pi 
         for omega in trajectory['robot orientation (deg)']]
print(omega)

plt.grid()
plt.plot(x_r, y_r, label="robot", marker="o")
plt.plot(x_d, y_d, label="detection", marker="o")
plt.legend()
plt.show()
