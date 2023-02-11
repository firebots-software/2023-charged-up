import numpy as np

data = [
    # (actual, yaw)
    (0.5, 2.67),
    (1, -6.68),
    (1.5, -9.98),
    #(2, -14.85),
    (3, -13.87)
]

tagHeight = 0.4699
cameraHeight = 0.16
cameraDegrees = 28

metersToTheoreticalDegrees = lambda meters : np.rad2deg(np.arctan((tagHeight-cameraHeight)/meters) - np.deg2rad(cameraDegrees))

def calculateConstant(data, meters2deg):
    xvals = []
    yvals = []
    for point in data:
        xvals.append(meters2deg(point[0]))
        yvals.append(point[1])
    
    coeffs = np.polyfit(np.array(yvals), np.array(xvals), 1)
    return str(coeffs.item(0)) + " * pitch + " + str(coeffs.item(1))

print(calculateConstant(data, metersToTheoreticalDegrees))