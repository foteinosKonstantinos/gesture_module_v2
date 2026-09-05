import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from matplotlib.transforms import Affine2D
import math

# https://matplotlib.org/stable/gallery/lines_bars_and_markers/marker_reference.html

# orientation w.r.t. to the magnetic north (+90 deg) and clockwise

trajectory = {'detection position (xy)': [(-531.4634950576074, 1588.2631235724964), (468.53650494236564, 2588.263123572487), (1468.536504942206, 5588.263123572451), (2468.5365049416814, 10588.263123572377), (3468.536504940402, 17588.263123572237), (4468.53650493782, 26588.263123572), (5468.536504933242, 37588.26312357163), (6468.536504925804, 50588.26312357106), (7468.5365049144975, 65588.26312357026), (8468.536504898155, 82588.26312356992), (9468.536504875443, 101588.2631235684), (10468.536504844906, 122588.26312356633)], 'robot position (xy)': [(0.0, 0.0), (999.9999999999999, 999.9999999999999), (1999.9999999999998, 3999.9999999999995), (3000.0, 9000.0), (3999.9999999999995, 15999.999999999998), (5000.0, 25000.000000000004), (6000.0, 36000.0), (7000.0, 49000.0), (7999.999999999999, 63999.99999999999), (9000.0, 81000.0), (10000.0, 100000.00000000001), (11000.0, 121000.0)], 'robot orientation (deg)': [330.0, 330.0, 330.0, 330.0, 330.0, 330.0, 330.0, 330.0, 330.0, 330.0, 330.0, 330.0]}


x_r = [xy[0]/1000 for xy in trajectory['robot position (xy)']]
y_r = [xy[1]/1000 for xy in trajectory['robot position (xy)']]

x_d = [xy[0]/1000 for xy in trajectory['detection position (xy)']]
y_d = [xy[1]/1000 for xy in trajectory['detection position (xy)']]

# radians
omega = [(-omega+90) #/ 180 * math.pi 
         for omega in trajectory['robot orientation (deg)']]
print(omega)

plt.grid()

plt.plot(x_r, y_r, label="robot", ls="-", marker=None, color="red")
for t in range(len(x_r)):
    plt.scatter([x_r[t]], [y_r[t]], marker=MarkerStyle(marker='d', transform=Affine2D().rotate_deg(omega[t]+90)), s=100, color="red")

plt.plot(x_d, y_d, label="detection", marker="o")
plt.legend()
plt.show()
