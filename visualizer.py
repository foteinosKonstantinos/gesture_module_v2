import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from matplotlib.transforms import Affine2D
import math

# https://matplotlib.org/stable/gallery/lines_bars_and_markers/marker_reference.html

# orientation w.r.t. to the magnetic north (+90 deg) and clockwise

# trajectory = {'detection position (xy)': [(327.5742574257487, 1537.2912115254553), (2327.5742574258616, 5537.291211525421), (3327.5742574261835, 10537.291211525355), (6327.5742574313745, 37537.29121152469), (7327.57425743595, 50537.29121152416), (8327.574257442919, 65537.29121152344), (9327.574257452985, 82537.29121152312), (10327.57425746697, 101537.29121152169), (11327.574257485794, 122537.29121151978), (12327.574257510445, 145537.29121151724), (13327.574257542043, 170537.2912115141), (14327.574257581791, 197537.29121151016), (15327.57425763098, 226537.2912115061), (16327.57425769098, 257537.29121150018), (17327.574257763325, 290537.2912114932)], 'robot position (xy)': [(0.0, 0.0), (1999.9999999999998, 3999.9999999999995), (3000.0, 9000.0), (6000.0, 36000.0), (7000.0, 49000.0), (7999.999999999999, 63999.99999999999), (9000.0, 81000.0), (10000.0, 100000.00000000001), (11000.0, 121000.0), (12000.0, 144000.0), (12999.999999999998, 169000.0), (14000.0, 196000.0), (15000.000000000004, 225000.0), (15999.999999999998, 255999.99999999997), (17000.0, 289000.0)], 'robot orientation (deg)': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}

trajectory = {'detection position (xy)': [(327.57425742574264, 1526.9999999999998), (1587.7584356971838, 2446.918765997064), (2057.43690110207, 3560.6841117022186), (1439.3158882968576, 3057.436901102685), (3672.425742573028, 2473.0), (4672.425742572722, 3473.0)], 'robot position (xy)': [(0.0, 0.0), (999.9999999996929, 1000.0000000000002), (1999.9999999993845, 1999.999999999999), (2999.9999999990773, 2999.9999999999995), (3999.9999999987704, 3999.9999999999995), (4999.999999998464, 5000.0)], 'robot orientation (deg)': [0.0, 10.0, -10.0, -100.0, -180.0, 180.0]}

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
