import math

pitchData = [
    # (actual, pitch)
    [0.5, 4.03],
    [1, -4.4],
    [1.5, -8.2],
    [2, -9.9],
    [3, -12]
]

yawData = [
    # ([distance to target straight, horizontal distance], yaw)
    [[0.2, 0.1], 2.67],
    [[0.3, 0.2], -6.68],
    [[0.5, 0.3], -9.98],
    [[0.2, -0.1], -13.87]
]

tagHeight = 0.4699
cameraHeight = 0.16
cameraDegrees = 28

def rad2Deg(x):
    return x*(180/math.pi)

def deg2Rad(x):
    return x*(math.pi/180)

metersToTheoreticalPitch = lambda meters : rad2Deg(math.atan2((tagHeight-cameraHeight), meters) - deg2Rad(cameraDegrees))
metersToTheoreticalYaw = lambda listXY : math.atan2(-listXY[1], listXY[0])

# Find the m value in y = mx that best fits the data
def findM(data, xindex, yindex):
    # We want to find the absolute min of the sum of (predicted - actual)^2
    # our predicted y value is m*x, so we're finding the min of the sum of (mx_n - y_n)^2

    # to do this, we can use the first derivative test to find the local mins
    # by setting the derivative of that to 0
    # the equation is in the form (m*x0 + y0)^2 + (m*x1 + y1)^2 + ... + (m*x_n + y_n)^2
    # so we can take each derivative seperately, then add them together.
    # d/dm (m*x_n-y_n)^2 = 2*(m*x_n-y_n)*x_n = 2m*(x_n)^2 - 2*x_n*y_n
    # 0 = (2m(x0)^2 - 2*x0*y0) + (2m(x1)^2 - 2*x1*y1) + ...

    # Now we can rearrange and m to solve for equation
    # 0 = (2m*(x0)^2 + 2m(x1)^2 + ...) - (2*x0*y0 + 2*x1*y1 + ...)
    # Factor out 2m from the first term, 2 from the second, /2 on both sides
    # 0 = m(x0^2 + x1^2 + ...) - (x0*y0 + x1*y1 + ...)
    # m = (x0*y0 + x1*y1 + ...)/(x0^2 + x1^2 + ...)

    numerator = 0 # accumulate x_n * y_n
    denominator = 0 # accumulate x_n^2

    for point in data:
        numerator += point[xindex] * point[yindex]
        denominator += point[xindex] * point[xindex]

    return numerator / denominator

def findLine(data, xindex, yindex):
    sumx = 0
    sumy = 0
    sumxy = 0
    sumxx = 0

    for point in data:
        sumx += point[xindex]
        sumy += point[yindex]
        sumxy += point[xindex] * point[yindex]
        sumxx += point[xindex] * point[xindex]
    
    m = (sumxy - (sumx * sumy) / len(data)) / (sumxx - (sumx * sumx) / len(data))
    b = (sumy - m * sumx) / len(data)

    return [m, b]

def convertIndex(data, index, converter):
    for p in range(len(data)):
        data[p][index] = converter(data[p][index])

convertIndex(pitchData, 0, metersToTheoreticalPitch)
convertIndex(yawData, 0, metersToTheoreticalYaw)

pitchline = findLine(pitchData, 1, 0)
yawline = findLine(yawData, 1, 0)

print("Pitch equation: " + str(pitchline[0]) + " * pitch + " + str(pitchline[1]))
print("Yaw equation: " + str(yawline[0]) + " * yaw + " + str(yawline[1]))
print()

print("// CONSTANTS:\nstatic final double AAGRIMS_CONSTANT = %0.10f;\nstatic final double YAJWINS_CONSTANT = %0.10f;\nstatic final double ARYAVS_CONSTANT = %0.10f;\nstatic final double RITVIKS_CONSTANT = %0.10f;" %(pitchline[0], pitchline[1], yawline[0], yawline[1]))

# y = -getDistance() * tan(yaw);
# y = -x * tan(yaw)
# meters to theoretical yaw = arctan(-y/x)