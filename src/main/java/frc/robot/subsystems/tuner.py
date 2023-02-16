import numpy as np

pitchData = [
    # (actual, pitch)
    (0.5, 2.67),
    (1, -6.68),
    (1.5, -9.98),
    #(2, -14.85),
    (3, -13.87)
]

yawData = [
    # ([distance to target straight, horizontal distance], yaw)
    ([0.2, 0.1], 2.67),
    ([0.3, 0.2], -6.68),
    ([0.5, 0.3], -9.98),
    ([0.2, -0.1], -13.87)
]

tagHeight = 0.4699
cameraHeight = 0.16
cameraDegrees = 28

metersToTheoreticalPitch = lambda meters : np.rad2deg(np.arctan2((tagHeight-cameraHeight), meters) - np.deg2rad(cameraDegrees))
metersToTheoreticalYaw = lambda listXY : np.arctan2(-listXY[1], listXY[0])

# Find the m value in y = mx that best fits the data
def findSlope(data, xindex, yindex, yconverter):
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
        numerator += point[xindex] * yconverter(point[yindex])
        denominator += point[xindex] * point[xindex]

    return numerator / denominator

def calculateConstant(data, xindex, yindex, yconverter):
    xvals = []
    yvals = []
    for point in data:
        xvals.append(point[xindex])
        yvals.append(yconverter(point[yindex]))
    
    coeffs = np.polyfit(np.array(xvals), np.array(yvals), 1)
    return str(coeffs.item(0)) + " * x + " + str(coeffs.item(1))


print("Pitch equation: " + calculateConstant(pitchData, 1, 0, metersToTheoreticalPitch))
print("Yaw equation: " + calculateConstant(yawData, 1, 0, metersToTheoreticalYaw))

# y = -getDistance() * tan(yaw);
# y = -x * tan(yaw)
# meters to theoretical yaw = arctan(-y/x)