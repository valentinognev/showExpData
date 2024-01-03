import math

NUM_POINTS = 16
M_PI = 3.14159265358979323846
VALIDLENGTH = 1.2
MAXTIMEDELAY = 500

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Circle:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

class refPoint_buf_t:
    def __init__(self, circBufferLen, last, first, data):
        self.circBufferLen = circBufferLen
        self.last = last
        self.first = first
        self.data = data

validAreaCenter = Point(0, 0)
validSimpleCenter = Point(0, 0)
hullAreaCenter = Point(0, 0)
hullSimpleCenter = Point(0, 0)
quadAreaCenter = Point(0, 0)
quadSimpleCenter = Point(0, 0)
circCenter = Point(0, 0)
validVec = [False] * NUM_POINTS
validPoints = [Point(0, 0)] * (NUM_POINTS + 1)
allPoints = [Point(0, 0)] * (NUM_POINTS + 1)
convexHullVec = [Point(0, 0)] * (NUM_POINTS + 1)
boundingQuad = [Point(0, 0)] * (NUM_POINTS + 1)

validAz = False
azimuth = 0  # integer angles 0-360

refPointsBuffer = refPoint_buf_t(circBufferLen=MAXTIMEDELAY, last=0, first=0, data=[0] * MAXTIMEDELAY)

def distance_p2p(p1, p2):
    dx = (p1.x - p2.x)
    dy = (p1.y - p2.y)
    return math.sqrt(dx*dx + dy*dy)

def crossProduct(p1, p2, p3):
    return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)

def comparePoints(p1, p2):
    orientation = crossProduct(p1[0], p1, p2)
    if orientation == 0:
        return (math.sqrt(pow(p1.x - p1[0].x, 2) + pow(p1.y - p1[0].y, 2)) >=
                math.sqrt(pow(p2.x - p1[0].x, 2) + pow(p2.y - p1[0].y, 2))) ? -1 : 1
    return -1 if orientation > 0 else 1

def calcConvexHull(points, numOfValidPoints, convexHull, hullSize):
    pivot = 0
    for i in range(1, numOfValidPoints):
        if points[i].y < points[pivot].y or (points[i].y == points[pivot].y and points[i].x < points[pivot].x):
            pivot = i

    newPoints = [points[(pivot+i) % numOfValidPoints] for i in range(numOfValidPoints)] + [points[0]]

    convexHull[0] = newPoints[0]
    convexHull[1] = newPoints[1]
    hullSize[0] = 2

    for i in range(2, numOfValidPoints + 1):
        res = crossProduct(convexHull[hullSize[0] - 2], convexHull[hullSize[0] - 1], newPoints[i])
        while hullSize[0] > 1 and res <= 0:
            hullSize[0] -= 1
            res = crossProduct(convexHull[hullSize[0] - 2], convexHull[hullSize[0] - 1], newPoints[i])
        convexHull[hullSize[0]] = newPoints[i]
        hullSize[0] += 1

def find_smallest_side(hull, N):
    min_side_length = float('inf')
    min_side_index = -1

    for i in range(N):
        p1 = hull[i]
        p2 = hull[(i + 1) % N]
        side_length = math.hypot(p2.x - p1.x, p2.y - p1.y)

        if side_length < min_side_length:
            min_side_length = side_length
            min_side_index = i

    return min_side_index

def find_intersection_point(hull, side_index, N):
    p1 = hull[side_index]
    p2 = hull[(side_index - 1 + N) % N]
    p3 = hull[(side_index + 1) % N]
    p4 = hull[(side_index + 2) % N]

    x_intersect = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) -
                    (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / \
                    ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x))

    y_intersect = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) -
                    (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / \
                    ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x))

    return Point(x_intersect, y_intersect)

def convexHull2Polygon(hull, N, n):
    sz = N
    while sz > n:
        min_side_index = find_smallest_side(hull, sz)
        intersection_point = find_intersection_point(hull, min_side_index, sz)
        hull[min_side_index] = intersection_point

        for i in range((min_side_index + 1) % sz, sz):
            hull[i] = hull[i + 1]

        sz -= 1

def calculateCenters(poly, sz, areaCenterX, areaCenterY, simpleCenterX, simpleCenterY):
    for i in range(sz):
        areaCenterX[0] += poly[i].x
        areaCenterY[0] += poly[i].y
    areaCenterX[0] /= sz
    areaCenterY[0] /= sz

    maxX = float('-inf')
    minX = float('inf')
    maxY = float('-inf')
    minY = float('inf')
    for i in range(sz):
        if poly[i].x > maxX:
            maxX = poly[i].x
        if poly[i].x < minX:
            minX = poly[i].x
        if poly[i].y > maxY:
            maxY = poly[i].y
        if poly[i].y < minY:
            minY = poly[i].y
    simpleCenterX[0] = (maxX + minX) / 2
    simpleCenterY[0] = (maxY + minY) / 2

def determineReferencePointFromQuad(boundingQuad, numOfValidPoints, convexHull, convHullSize, referencePoint, numOfValidSides):
    validSide = [False] * 4
    for i in range(4):
        validSide[i] = numOfValidPoints[i] >= 3
        if validSide[i]:
            numOfValidSides[0] += 1

    validReferencePoint = False

    if numOfValidSides[0] == 2:
        if (validSide[0] and not validSide[1] and validSide[2] and not validSide[3]) or \
           (not validSide[0] and validSide[1] and not validSide[2] and validSide[3]):
            validReferencePoint = False

        for i in range(4):
            if validSide[i] and validSide[(i + 1) % 4]:
                referencePoint[0] = boundingQuad[(i + 1) % 4]
                validReferencePoint = True
    elif numOfValidSides[0] == 3:
        for i in range(4):
            if validSide[i] and validSide[(i + 1) % 4] and validSide[(i + 2) % 4]:
                referencePoint.x = (boundingQuad[(i + 1) % 4].x + boundingQuad[(i + 2) % 4].x) / 2
                referencePoint.y = (boundingQuad[(i + 1) % 4].y + boundingQuad[(i + 2) % 4].y) / 2
                validReferencePoint = True
    elif numOfValidSides[0] == 4:
        calculateCenters(convexHull, convHullSize - 1, hullAreaCenter.x, hullAreaCenter.y, hullSimpleCenter.x,
                          hullSimpleCenter.y)
        referencePoint[0] = hullSimpleCenter
        validReferencePoint = True
    else:
        validReferencePoint = False

    return validReferencePoint

def get_reference_point_in_shaft(allPoints, c, yaw, cmd_x, cmd_y):
    cmd_x[0] = 0.0
    cmd_y[0] = 0.0

    points = [Point(0, 0)] * NUM_POINTS
    theta = 0.0
    delta = 2.0 * M_PI / NUM_POINTS

    cosYaw = math.cos(yaw)
    sinYaw = math.sin(yaw)

    for i in range(NUM_POINTS):
        points[i].x = allPoints[i].x * cosYaw - allPoints[i].y * sinYaw
        points[i].y = allPoints[i].x * sinYaw + allPoints[i].y * cosYaw

    largest_circle = Circle(center=Point(0, 0), radius=0)

    for i in range(NUM_POINTS):
        for j in range(i + 1, NUM_POINTS):
            center = Point((points[i].x + points[j].x) / 2, (points[i].y + points[j].y) / 2)

            radius = 10.0
            for k in range(NUM_POINTS):
                dist = distance_p2p(center, points[k])
                if dist < radius:
                    radius = dist

            if radius > largest_circle.radius and distance_p2p(center, c) < 0.2:
                largest_circle.center = center
                largest_circle.radius = radius

    cmd_x[0] = largest_circle.center.x
    cmd_y[0] = largest_circle.center.y

def getReferencePointFromRanges(ranges, referencePoint, numOfSides):
    angles = [0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5]
    sinAngles = [0, 0.3826834324, 0.7071067812, 0.9238795325, 1, 0.9238795325, 0.7071067812, 0.3826834324, 0, -0.3826834324,
                 -0.7071067812, -0.9238795325, -1, -0.9238795325, -0.7071067812, -0.3826834324]
    cosAngles = [1, 0.9238795325, 0.7071067812, 0.3826834324, 0, -0.3826834324, -0.7071067812, -0.9238795325, -1,
                 -0.9238795325, -0.7071067812, -0.3826834324, 0, 0.3826834324, 0.7071067812, 0.9238795325]

    validPointsSize = 0

    for i in range(NUM_POINTS):
        allPoints[i].x = ranges[i] * cosAngles[i]
        allPoints[i].y = ranges[i] * sinAngles[i]

        validVec[i] = ranges[i] < VALIDLENGTH and ranges[i] > 1e-5
        validPoints[validPointsSize].x = 0
        validPoints[validPointsSize].y = 0

        if not validVec[i]:
            continue

        validPoints[validPointsSize].x = allPoints[i].x
        validPoints[validPointsSize].y = allPoints[i].y
        validPointsSize += 1

    if validPointsSize < 3:
        return False

    hullSize = [0]
    calcConvexHull(validPoints, validPointsSize, convexHullVec, hullSize)

    for i in range(hullSize[0]):
        boundingQuad[i] = convexHullVec[i]
    for i in range(hullSize[0], NUM_POINTS + 1):
        convexHullVec[i] = Point(0, 0)

    convexHull2Polygon(boundingQuad, hullSize[0], 4)

    testX = [validPoints[i].x for i in range(NUM_POINTS)]
    testY = [validPoints[i].y for i in range(NUM_POINTS)]
    testhullX = [convexHullVec[i].x for i in range(NUM_POINTS + 1)]
    testhullY = [convexHullVec[i].y for i in range(NUM_POINTS + 1)]
    testquadX = [boundingQuad[i].x for i in range(NUM_POINTS + 1)]
    testquadY = [boundingQuad[i].y for i in range(NUM_POINTS + 1)]

