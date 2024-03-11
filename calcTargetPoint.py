import numpy as np
import matplotlib.pyplot as plt
import csv
from numpy import cos, sin
from math import atan2
from copy import deepcopy

VALIDLENGTH = 1.0

def generate_ray_hit_points(x, y, vertices, N, noise_std=0.1):
    # Generate equally spaced angles
    angles = np.linspace(0, 2*np.pi, N, endpoint=False)

    # Calculate ray hit points on the edges of the quadrilateral with Gaussian noise
    hit_points = []
    for angle in angles:
        for i in range(len(vertices)):
            # Get current and next vertex
            current_vertex = vertices[i]
            next_vertex = vertices[(i + 1) % len(vertices)]

            # Parametric equation of the line
            hit_x = (1 - angle / (2 * np.pi)) * \
                current_vertex[0] + (angle / (2 * np.pi)) * next_vertex[0]
            hit_y = (1 - angle / (2 * np.pi)) * \
                current_vertex[1] + (angle / (2 * np.pi)) * next_vertex[1]

            # Calculate the direction of the ray
            ray_dir_x = next_vertex[0] - current_vertex[0]
            ray_dir_y = next_vertex[1] - current_vertex[1]
            norm = np.sqrt(ray_dir_x**2 + ray_dir_y**2)

            # Add Gaussian noise along the direction of the ray
            noise_x = np.random.normal(0, noise_std)
            noise_y = np.random.normal(0, noise_std)

            hit_x += noise_x * (ray_dir_x / norm)
            hit_y += noise_y * (ray_dir_y / norm)

            hit_points.append((hit_x, hit_y))

    return hit_points

#############################################################################################

def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0
    return 1 if val > 0 else -1

def crossProduct(p1, p2, p3):
    return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

def calcConvexHull(points):
    numOfValidPoints = len(points)
    pivot = 0
    for i in range(1, numOfValidPoints):
        if points[i,1] < points[pivot,1] or \
          (points[i,1] == points[pivot,1] and points[i,0] < points[pivot,0]):
            pivot = i

    newPoints = [points[(pivot+i) % numOfValidPoints] for i in range(numOfValidPoints)]
    newPoints = newPoints + [newPoints[0]]
    newPoints=np.array(newPoints)

    convexHull=np.zeros(np.shape(points))
    convexHull[0,:] = newPoints[0]
    convexHull[1,:] = newPoints[1]
    hullSize = 2

    for i in range(2, numOfValidPoints + 1):
        res = crossProduct(convexHull[hullSize - 2], convexHull[hullSize - 1], newPoints[i])
        while hullSize > 1 and res <= 0:
            hullSize -= 1
            res = crossProduct(convexHull[hullSize - 2], convexHull[hullSize - 1], newPoints[i])
        convexHull[hullSize] = newPoints[i]
        hullSize += 1

    return convexHull, hullSize
#######################################################################################

def plot_convex_hull(points, convex_hull):
    x, y = zip(*points)
    hull_x, hull_y = zip(*convex_hull)

    plt.scatter(x, y, marker='o', label='Points')
    plt.plot(hull_x + (hull_x[0],), hull_y + (hull_y[0],),
             color='red', linestyle='-', linewidth=2, label='Convex Hull')

    plt.title('Convex Hull Calculation')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.legend()
    plt.show()

#######################################################################################

def find_intersection_point(hull, side_index):
    p1 = hull[side_index]
    p2 = hull[(side_index - 1) % len(hull)]
    p3 = hull[(side_index + 1) % len(hull)]
    p4 = hull[(side_index + 2) % len(hull)]

    # Extend the sides until they intersect
    x_intersect = ((p1[0] * p2[1] - p1[1] * p2[0]) * (p3[0] - p4[0]) -
                   (p1[0] - p2[0]) * (p3[0] * p4[1] - p3[1] * p4[0])) / \
                  ((p1[0] - p2[0]) * (p3[1] - p4[1]) -
                   (p1[1] - p2[1]) * (p3[0] - p4[0]))

    y_intersect = ((p1[0] * p2[1] - p1[1] * p2[0]) * (p3[1] - p4[1]) -
                   (p1[1] - p2[1]) * (p3[0] * p4[1] - p3[1] * p4[0])) / \
                  ((p1[0] - p2[0]) * (p3[1] - p4[1]) -
                   (p1[1] - p2[1]) * (p3[0] - p4[0]))

    intersection_point = np.array([x_intersect, y_intersect])
    return intersection_point

#######################################################################################

def find_smallest_side(hull):
    min_side_length = float('inf')
    min_side_index = -1

    for i in range(len(hull)):
        p1 = hull[i]
        p2 = hull[(i + 1) % len(hull)]
        side_length = np.linalg.norm(p2 - p1)

        if side_length < min_side_length:
            min_side_length = side_length
            min_side_index = i

    return min_side_index

#######################################################################################

def convexHull2Polygon(hull, N):

    while len(hull) > N:
        min_side_index = find_smallest_side(hull)
        intersection_point = find_intersection_point(hull, min_side_index)

        # Remove the points of the shortest side
        hull = np.delete(
            hull, [min_side_index, (min_side_index + 1) % len(hull)], axis=0)

        # Insert the new point
        hull = np.insert(hull, min_side_index % len(hull), intersection_point, axis=0)

    return hull

#######################################################################################

def calculateCenters(poly):
    areaCenter = np.mean(poly, 0)

    maxP = np.max(poly, 0)
    minP = np.min(poly, 0)

    simpleCenter = (maxP + minP) / 2

    return areaCenter, simpleCenter

#######################################################################################

def readMR18DataFromCSV(file_path, timestart=0, timeend=1000000000):
    # Initialize an empty list to store data
    data = []

    # Open the CSV file and read its content using csv.DictReader
    with open(file_path, newline='') as csvfile:
        # Create a DictReader object
        csv_reader = csv.DictReader(csvfile)

        # Iterate over rows and extract data
        for row in csv_reader:
            data.append(row)

    # Extract column names from the header
    columns = list(data[0].keys())

    # Create a NumPy 2D array

    tick = np.array([float(row['tick']) for row in data])
    indmstart = np.where(tick > timestart)[0][0]
    indmend = np.where(tick < timeend)[0][-1]
    # motor = np.array([float(row['state_machine.SM_St_log']) for row in data])
    # indmstart = np.where(motor>12)[0][0]
    
    mr18Data = []
    mr18Data.append(np.array([float(row['mr18.m0']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m1']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m2']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m3']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m4']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m5']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m6']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m7']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m8']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m9']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m10']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m11']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m12']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m13']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m14']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m15']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m16']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m17']) for row in data]))
    mr18Data= np.array(mr18Data).transpose()

    newTick = np.linspace(tick[indmstart], tick[indmend], indmend-indmstart)
    newmr18Data = np.zeros((indmend-indmstart, 18))
    for i in range(18):
        newmr18Data[:,i] = np.interp(newTick, tick[indmstart:indmend], mr18Data[indmstart:indmend,i])
    
    return newTick/1000, newmr18Data/1000

    # return tick[indmstart:indmend]/1000, mr18Data[indmstart:indmend,:]/1000

#######################################################################################
def simple_lowpass_filter(data, cutoff_frequency, samp_rate):
    """
    Simple low-pass filter using a moving average.

    Parameters:
    - data: The input data array.
    - cutoff_frequency: The cutoff frequency for the low-pass filter.
    - samp_rate: The sampling rate of the data.

    Returns:
    - filtered_data: The filtered output data array.
    """

    # Calculate the filter window size
    window_size = int(samp_rate / cutoff_frequency)

    # Initialize the filtered data array
    filtered_data = np.zeros_like(data)

    # Apply the moving average filter
    for i in range(window_size, len(data)):
        filtered_data[i] = np.mean(data[i - window_size:i])

    return filtered_data

#######################################################################################

inds = np.array(range(1,48,3))
indr = np.array(range(0,48,3))
# Vertices of the quadrilateral
angles = np.linspace(0, 2*np.pi, 16, endpoint=False)
# times, mt18data = readMR18DataFromCSV('/home/valentin/Downloads/sd_33_cf2410chimney2.csv', timestart=155000, timeend=190000)
times, mt18data = readMR18DataFromCSV('/home/valentin/crazyflie/Recordings/tube/sd_33_cf2410chimney2.csv', timestart=155000, timeend=190000)
num_rays = 8  # Number of equally spaced rays

centers = np.zeros((len(times), 12))
distance = np.zeros((len(times), 2))

for i in range(len(times)):
    ranges = mt18data[i,0:16]
    #    hit_points = generate_ray_hit_points(point_x, point_y, quad_vertices, num_rays, noise_std=0.1)
    allPoints = np.array([ranges*np.cos(angles), ranges*np.sin(angles)]).transpose() 
    validPointInds = ranges < VALIDLENGTH*2
    indV = np.where(validPointInds)[0]
    indNV = np.where(~validPointInds)[0]

    validPoints = allPoints[validPointInds,:]
    convex_hull, hull_size = calcConvexHull(validPoints)  #graham_scan(hit_points)
    convex_hullarr = np.array(convex_hull)[:(hull_size-1)]
    quad_vertices = convexHull2Polygon(convex_hullarr,4)

    [validAreaCenter, validSimpleCenter] = calculateCenters(validPoints)
    [hullAreaCenter, hullSimpleCenter] = calculateCenters(convex_hullarr)
    [quadAreaCenter, quadSimpleCenter] = calculateCenters(quad_vertices)

    centers[i,:] = np.array([validAreaCenter[0], validAreaCenter[1], \
                             validSimpleCenter[0], validSimpleCenter[1], \
                             hullAreaCenter[0], hullAreaCenter[1], \
                             hullSimpleCenter[0], hullSimpleCenter[1], \
                             quadAreaCenter[0]*0, quadAreaCenter[1]*0, \
                             quadSimpleCenter[0]*0, quadSimpleCenter[1]*0])
    distance[i,:] = np.array([ranges[0]-0.3, ranges[4]-0.3])

    if times[i] > 174 and times[i] < 175:
        raysV = np.zeros((48, 2))+np.nan
        raysV[inds[indV], 0] = ranges[indV]*cos(angles[indV])
        raysV[inds[indV], 1] = ranges[indV]*sin(angles[indV])
        raysV[indr[indV], 0] = ranges[indV]*cos(angles[indV])*0
        raysV[indr[indV], 1] = ranges[indV]*sin(angles[indV])*0

        raysNV = np.zeros((48, 2))+np.nan
        raysNV[inds[indNV], 0] = ranges[indNV]*cos(angles[indNV])
        raysNV[inds[indNV], 1] = ranges[indNV]*sin(angles[indNV])
        raysNV[indr[indNV], 0] = ranges[indNV]*cos(angles[indNV])*0
        raysNV[indr[indNV], 1] = ranges[indNV]*sin(angles[indNV])*0
        
        plt.figure(1)
        plt.plot(raysV[:, 0], raysV[:, 1], 'b', linewidth=1, label='Valid rays')
        plt.plot(raysNV[:, 0], raysNV[:, 1], 'r', linewidth=1, label='Invalid rays')
        plt.plot(convex_hullarr[:, 0], convex_hullarr[:, 1], 'g', linewidth=1, label='Convex Hull')
        # plt.plot(quad_vertices[:, 0], quad_vertices[:, 1], 'm', linewidth=1, label='Bounding Quad')
        plt.plot(validAreaCenter[0], validAreaCenter[1], 'bo', label='Valid Area Center')
        plt.plot(validSimpleCenter[0], validSimpleCenter[1], 'bx', label='Valid Simple Center')
        plt.plot(hullAreaCenter[0], hullAreaCenter[1], 'go', label='Hull Area Center')
        plt.plot(hullSimpleCenter[0], hullSimpleCenter[1], 'gx', label='Hull Simple Center')
        # plt.plot(quadAreaCenter[0], quadAreaCenter[1], 'mo', label='Quad Area Center')
        # plt.plot(quadSimpleCenter[0], quadSimpleCenter[1], 'mx', label='Quad Simple Center')
        
        plt.title('Quadrilateral, Ray Start Point, and Rays with Gaussian Noise')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()
samplingRate = 1./np.mean(np.diff(times))
cutOffFreq = 10
for i in range(centers.shape[1]):
    centers[:,i] = simple_lowpass_filter(centers[:,i], cutOffFreq/2, samplingRate)

for i in range(distance.shape[1]):
    distance[:,i] = simple_lowpass_filter(centers[:,i], cutOffFreq, samplingRate)

plt.figure(2)
plt.subplot(2,1,1)
plt.plot(times, centers[:,0], 'b', linewidth=1, label='Valid Area Center')
plt.plot(times, centers[:,2], 'r', linewidth=1, label='Valid Simple Center')
plt.plot(times, centers[:,4], 'g', linewidth=1, label='Hull Area Center')
plt.plot(times, centers[:,6], 'm', linewidth=1, label='Hull Simple Center')
plt.plot(times, centers[:,8], 'c', linewidth=1, label='Quad Area Center')
plt.plot(times, centers[:,10], 'y', linewidth=1, label='Quad Simple Center')
plt.plot(times, distance[:,0], 'b', linewidth=1, label='simpleDistance')
plt.title('Centers X')
plt.legend()
plt.grid(True)

plt.subplot(2,1,2)
plt.plot(times, centers[:,1], 'b', linewidth=1, label='Valid Area Center')
plt.plot(times, centers[:,3], 'r', linewidth=1, label='Valid Simple Center')
plt.plot(times, centers[:,5], 'g', linewidth=1, label='Hull Area Center')
plt.plot(times, centers[:,7], 'm', linewidth=1, label='Hull Simple Center')
plt.plot(times, centers[:,9], 'c', linewidth=1, label='Quad Area Center')
plt.plot(times, centers[:,11], 'y', linewidth=1, label='Quad Simple Center')
plt.plot(times, distance[:,1], 'b', linewidth=1, label='simpleDistance')
plt.title('Centers Y')
plt.legend()
plt.grid(True)
plt.show()

# plt.figure(3)
# plt.plot(times[:-1], np.diff(times), 'b', linewidth=1, label='Sampling Time')
# plt.title('Sampling Time')
# plt.show()
a=1
# Generate some random points forming a convex hull
# import numpy as np
# import matplotlib.pyplot as plt
# from math import atan2
# from scipy.spatial import ConvexHull


# def orientation(p, q, r):
#     val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
#     if val == 0:
#         return 0
#     return 1 if val > 0 else -1

# def convex_hull(points):
#     n = len(points)
#     if n < 3:
#         return points

#     pivot = min(points, key=lambda point: (point[1], point[0]))
#     sorted_points = sorted(points, key=lambda point: (
#         atan2(point[1] - pivot[1], point[0] - pivot[0]), point))
#     hull = [pivot, sorted_points[0], sorted_points[1]]

#     for i in range(2, n):
#         while len(hull) > 1 and orientation(hull[-2], hull[-1], sorted_points[i]) != -1:
#             hull.pop()
#         hull.append(sorted_points[i])

#     return hull


# def find_smallest_side(hull):
#     min_side_length = float('inf')
#     min_side_index = -1

#     for i in range(len(hull.vertices)):
#         p1 = hull.points[hull.vertices[i]]
#         p2 = hull.points[hull.vertices[(i + 1) % len(hull.vertices)]]
#         side_length = np.linalg.norm(p2 - p1)

#         if side_length < min_side_length:
#             min_side_length = side_length
#             min_side_index = i

#     return min_side_index


# def find_intersection_point(hull, side_index):
#     p1 = hull[side_index]
#     p2 = hull[(side_index - 1) % len(hull)]
#     p3 = hull[(side_index + 1) % len(hull)]
#     p4 = hull[(side_index + 2) % len(hull)]

#     # Extend the sides until they intersect
#     x_intersect = ((p1[0] * p2[1] - p1[1] * p2[0]) * (p3[0] - p4[0]) -
#                    (p1[0] - p2[0]) * (p3[0] * p4[1] - p3[1] * p4[0])) / \
#                   ((p1[0] - p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] - p4[0]))

#     y_intersect = ((p1[0] * p2[1] - p1[1] * p2[0]) * (p3[1] - p4[1]) -
#                    (p1[1] - p2[1]) * (p3[0] * p4[1] - p3[1] * p4[0])) / \
#                   ((p1[0] - p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] - p4[0]))

#     intersection_point = np.array([x_intersect, y_intersect])
#     return intersection_point

# def find_smallest_side(hull):
#     min_side_length = float('inf')
#     min_side_index = -1

#     for i in range(len(hull)):
#         p1 = hull[i]
#         p2 = hull[(i + 1) % len(hull)]
#         side_length = np.linalg.norm(p2 - p1)

#         if side_length < min_side_length:
#             min_side_length = side_length
#             min_side_index = i

#     return min_side_index

# def convexHull2Polygon(hull, N):

#     while len(hull) > N:
#         min_side_index = find_smallest_side(hull)
#         intersection_point = find_intersection_point(hull, min_side_index)

#         # Remove the points of the shortest side
#         hull = np.delete(hull, [min_side_index, (min_side_index + 1) % len(hull)], axis=0)

#         # Insert the new point
#         hull = np.insert(hull, min_side_index, intersection_point, axis=0)

#     return hull

# # Generate some random points forming a convex hull
# np.random.seed(0)
# points = np.random.rand(10, 2)
# convex_hull_points = convex_hull(points)
# convex_hull_points = np.array(convex_hull_points)[1:]
# plt.scatter(points[:, 0], points[:, 1], label='Convex Hull Points')
# plt.plot(np.append(convex_hull_points[:, 0], convex_hull_points[0, 0]),
#          np.append(convex_hull_points[:, 1], convex_hull_points[0, 1]),
#          label='Convex Hull', color='blue')
# plt.show()
# # Find the minimum area bounding quad
# min_area_quad = convexHull2Polygon(convex_hull_points, 4)


# min_area_quad = np.array(min_area_quad)
# # Plotting
# plt.scatter(points[:, 0], points[:, 1], label='Convex Hull Points')
# plt.plot(np.append(convex_hull_points[:, 0], convex_hull_points[0, 0]),
#          np.append(convex_hull_points[:, 1], convex_hull_points[0, 1]),
#          label='Convex Hull', color='blue')
# plt.scatter(np.append(min_area_quad[:, 0], min_area_quad[0, 0]),
#          np.append(min_area_quad[:, 1], min_area_quad[0, 1]),
#          label='Minimum Area Bounding Quad', color='red')
# plt.legend()
# plt.show()


# a=1