import math
import numpy as np

goal_latitude = 35.7417229
goal_longtitude = 140.0101197
goal_altitude = 45.0

difference_lat = 111316.2056
difference_lon = math.cos(goal_latitude/180*math.pi)*math.pi*6378.137/180*1000

point1 = -10+1j*10
point2 = -5*(1+math.sqrt(5))+1j*(-5*(math.sqrt(5)-1))
point3 = 1j*(-10*(1+math.sqrt(5)))
point = np.empty(3)
point[0] = point1
point[1] = point2
point[2] = point3

direction = "E"

if direction == "N":
    point *= 1j
elif direction == "W":
    point *= -1
elif direction == "S":
    point *= -1j

point_gps = []
for i in range(3):
    point_gps.append([])
    point_gps[i] = np.real(point[i])/difference_lat+goal_latitude
    point_gps[i].append(np.imag(point[i])/difference_lon+goal_longtitude)

for i in range(3):
    print(point_gps[i][0])
    print(point_gps[i][1])