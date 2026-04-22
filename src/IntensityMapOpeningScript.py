from Listener import OccupancyGridMap
import numpy as np
import os

def open():
    abspath = os.getcwd() + "/data/"
    filename = str(input("Enter filename (without extension): "))
    abspath = "data/" + filename + ".csv"
    print(abspath)
    rawIntensities = np.genfromtxt(abspath, delimiter=',')
    OGM = OccupancyGridMap(3,3)
    OGM.rawIntensities = rawIntensities
    OGM.display() #blocking
    averagedIntensities = OGM.spatial_average(6)
    averagedOGM = OccupancyGridMap(3,3)
    averagedOGM.rawIntensities = averagedIntensities
    averagedOGM.save("AverageTest")
    averagedOGM.display()

if __name__ == '__main__':
    open()
else:
    pass
