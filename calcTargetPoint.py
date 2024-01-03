import numpy as np
import matplotlib.pyplot as plt
import csv
from math import atan2
from copy import deepcopy


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

def calcConvexHull(points, numOfValidPoints):
    pivot = 0
    for i in range(1, numOfValidPoints):
        if points[i,1] < points[pivot,1] or \
          (points[i,1] == points[pivot,1] and points[i,0] < points[pivot,0]):
            pivot = i

    newPoints = [points[(pivot+i) % numOfValidPoints] for i in range(numOfValidPoints)] + [points[0]]

    convexHull=[]
    convexHull.append(newPoints[0])
    convexHull.append(newPoints[1])
    hullSize[0] = 2

    for i in range(2, numOfValidPoints + 1):
        res = crossProduct(convexHull[hullSize[0] - 2], convexHull[hullSize[0] - 1], newPoints[i])
        while hullSize[0] > 1 and res <= 0:
            hullSize[0] -= 1
            res = crossProduct(convexHull[hullSize[0] - 2], convexHull[hullSize[0] - 1], newPoints[i])
        convexHull[hullSize[0]] = newPoints[i]
        hullSize[0] += 1

    return convexHull
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

    return tick[indmstart:indmend]/1000, mr18Data[indmstart:indmend,:]

#######################################################################################

# Example usage
point_x = 2.0  # x-coordinate of the point
point_y = 3.0  # y-coordinate of the point
# Vertices of the quadrilateral
angles = np.linspace(0, 2*np.pi, 16, endpoint=False)
times, mt18data = readMR18DataFromCSV('/home/valentin/crazyflie/Recordings/tube/sd_33_cf2410chimney2.csv', timestart=155000, timeend=190000)
num_rays = 8  # Number of equally spaced rays

for i in range(len(times)):
    ranges = mt18data[i,0:16]
    #    hit_points = generate_ray_hit_points(point_x, point_y, quad_vertices, num_rays, noise_std=0.1)
    hit_points = np.array([ranges*np.cos(angles), ranges*np.sin(angles)]).transpose() 

    convex_hull = graham_scan(hit_points)
    convex_hullarr = np.array(convex_hull)
    quad_vertices = convexHull2Polygon(convex_hullarr,4)

    # Plotting the quadrilateral, ray start point, rays, and ray hit points
    quad_x, quad_y = zip(*quad_vertices, quad_vertices[0])  # Closing the loop

    plt.figure()
    plt.plot(quad_x, quad_y, 'b-', label='Quadrilateral')
    plt.plot(point_x, point_y, 'go', label='Ray Start Point')

    for hit_x, hit_y in hit_points:
        plt.plot([point_x, hit_x], [point_y, hit_y], 'k--', alpha=0.5)
        plt.plot(hit_x, hit_y, 'ro')

    plt.title('Quadrilateral, Ray Start Point, and Rays with Gaussian Noise')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.legend()
    plt.grid(True)
    plt.show()
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