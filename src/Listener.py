#!/usr/bin/env python
import rospy, ros_numpy, math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String, Int32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class OccupancyGridMap(object):

    """This class defines an occupancy grid map (OGM) and the methods for creating, showing, updating it and saving it"""

    def __init__(self, startingXmax, startingYmax, selfNoiseState='n', resolution=3):
        self.selfNoiseState = selfNoiseState #can be w for write, r for read or n for none
        self.resolution = resolution
        self.width = 2*startingXmax*10**(self.resolution-1)
        self.height = 2*startingYmax*10**(self.resolution-1)
        self.rawIntensities = np.zeros((self.height,self.width))
        self.range = None 
        self.selfNoiseFunctionObj = None #object instantiated when range is determined by the first scan slice recieved
    
    def writeSelfNoise(self, pc_array):
        bins = []
        for point in pc_array:
            bins.append(point[3]) #assume pc_array is ordered by range
        self.selfNoiseFunctionObj.range = self.range
        self.selfNoiseFunctionObj.add_bins_to_collection(bins)
        if rospy.get_rostime().secs%10 == 0:
            self.selfNoiseFunctionObj.save()
    
    def readSelfNoiseAndModifyBins(self, pc_array):
        self.selfNoiseFunctionObj.load()
        # rospy.loginfo(self.selfNoiseFunctionObj.result)
        NoiseFunctionRanges = self.selfNoiseFunctionObj.result['range']
        NoiseFunctionIntensities = self.selfNoiseFunctionObj.result['noiseIntensity']
        modifiedpc_array = []
        for i in range(len(pc_array)):
            point = pc_array[i]
            rangeVal = round((point[0]**2+point[1]**2)**0.5,2)
            # rospy.loginfo(rangeVal)
            for j in range(len(NoiseFunctionRanges)):
                if round(NoiseFunctionRanges[j],2) == rangeVal:
                    modifiedIntensity = point[3] - (NoiseFunctionIntensities[i]*1.25)
                    if modifiedIntensity < 0:
                        modifiedIntensity = 0
                    break
            modifiedPoint = [point[0], point[1], point[2], modifiedIntensity]
            modifiedpc_array.append(modifiedPoint)
        return modifiedpc_array
    

    def update(self, pc_array_msg):
        #initially, this assumes that the pointcloud is observed from the origin
        #the origin is in the centre of the image
        pc_array = self.pc_msg_to_array(pc_array_msg)
        if self.range == None:
            lastPoint = pc_array[-1]
            self.range = round((lastPoint[0]**2+lastPoint[1]**2)**0.5,1)
            self.selfNoiseFunctionObj = SelfNoiseFunction(range=self.range)
            rospy.loginfo("range is now: " + str(self.range) + "NoiseFunc instantiated with this range")
        if self.selfNoiseState == 'w':
            self.writeSelfNoise(pc_array)
        elif self.selfNoiseState == 'r' and self.range != None:
            pc_array = self.readSelfNoiseAndModifyBins(pc_array)
        for i in range(0, len(pc_array)-1):
            point = pc_array[i]
            #a point is a 1x4 array, with fields x,y,z,intensity
            #goal is to place the intensity at the correct place on the OGM
            xPos, yPos = self.positionConverter(point[0], point[1], self.resolution)
            #rospy.loginfo("xPos: {}, yPos: {}, int: {}".format(xPos, yPos, point[3]))
            self.rawIntensities[xPos][yPos] = point[3]
    

    def show(self):
        #normalisedIntensities = self.intensityNormaliser()
        plt.imshow(self.rawIntensities, cmap='gray')
        plt.draw()
        plt.pause(0.001)

    def display(self):
        ## A version of show that keeps plot open - for use with opening saved raw intensity files
        #normalisedIntensities = self.intensityNormaliser(self.rawIntensities)
        plt.imshow(self.rawIntensities, cmap='gray')
        plt.show()

    
    def save(self, filename):
        rawFilename = filename+"Raw"
        imageFilename = filename
        avgImageFilename = filename+"Avg"
        #save the raw intensities as a csv
        pathfromIP_ws = "src/pc_talk_listen/src/data/"
        path = pathfromIP_ws + rawFilename + ".csv"
        np.savetxt(path, self.rawIntensities, delimiter=',')
        rospy.loginfo("Raw intensities saved as "+path)
        #save the sonar image as a png
        plt.imsave(pathfromIP_ws+imageFilename+".png", self.rawIntensities, cmap='gray')
        rospy.loginfo("Sonar image saved as "+pathfromIP_ws+imageFilename+".png")
        #save the averaged sonar image as a png
        averagedIntensities = self.spatial_average(6)
        plt.imsave(pathfromIP_ws+avgImageFilename+".png", averagedIntensities, cmap='gray')
        rospy.loginfo("Averaged sonar image saved as "+pathfromIP_ws+avgImageFilename+".png")

    def pc_msg_to_array(self, pointcloud_msg):
        pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud_msg)
        return pc_array

    def positionConverter(self, x, y, resolution):
        #takes a pair of world coordinates (relative to sonar) and converts to coordinates in image array
        xOrigin = self.width/2
        yOrigin = self.height/2
        xRound, yRound = round(x, self.resolution), round(y, self.resolution)
        xPixel, yPixel = int(xRound*10**(resolution-1)), int(yRound*10**(resolution-1))
        xOut = xOrigin+xPixel
        yOut = yOrigin+yPixel
        return xOut,yOut
    
    def findMinAndMaxIntensity(self):
        minInte,maxInte = self.rawIntensities[0][0],self.rawIntensities[0][0]
        for row in self.rawIntensities:
            for intensity in row:
                if intensity < minInte:
                    minInte = intensity
                elif intensity > maxInte:
                    maxInte = intensity
        return minInte, maxInte


    def intensityNormaliser(self):
        minI, maxI = self.findMinAndMaxIntensity()
        intensityRange = maxI-minI
        normalisedIntensities = []
        for row in self.rawIntensities:
            newRow = []
            for intensity in row:
                normalisedIntensity = ((intensity - minI)/intensityRange)*255
                newRow.append(normalisedIntensity)
            normalisedIntensities.append(newRow)
        return np.array(normalisedIntensities)
    
    def spatial_average(self, blockSize):
        #A function that splits the OGM into square blocks of a given size, and assigns the intensity
        # of each pixel in the block to be the average (for now, consider other distributions) in the block
        averagedOGM = np.zeros(np.shape(self.rawIntensities),dtype=np.int32)
        start = int(math.ceil(float(blockSize)/2))
        for i in range(start, len(self.rawIntensities)-blockSize, blockSize):
            row = self.rawIntensities[i]
            for j in range(start, len(row)-blockSize, blockSize):
                mean = self.averaging_function(self.get_Block_Values(blockSize, [i,j]))
                if mean <0:
                    print("Negative mean at ", i, j)
                    mean = mean *-1
                #print("The mean of block at %i, %i, is %i",i,j,mean)
                averagedOGM = self.set_Block_Values(blockSize, [i,j], averagedOGM, mean)
        
        return averagedOGM
    
    def find_top_n_values(self, n, array):
        length = len(array)
        sortedArray = sorted(array)
        for val in sortedArray[length-n:]:
            if val < 0:
                print(sortedArray[length-n:])
        return sortedArray[length-n:]

    def averaging_function(self, blockValues):
        n = int(len(blockValues)/4)
        return np.mean(self.find_top_n_values(n,blockValues))
    
    def get_Block_Values(self, blockSize, coords):
        blockRange = int(math.floor(float(blockSize)/2))
        blockValues = []
        for row in self.rawIntensities[(coords[0]-blockRange):(coords[0]+blockRange+1)]:
            for value in row[coords[1]-blockRange:coords[1]+blockRange+1]:
                blockValues.append(value)
        return blockValues
    
    def set_Block_Values(self, blockSize, coords, inputOGM, num):
        blockRange = int(math.floor(float(blockSize)/2))
        outputOGM = inputOGM.view()
        for row in range(coords[0]-blockRange,coords[0]+blockRange+1):
            for col in range(coords[1]-blockRange,coords[1]+blockRange+1):
                outputOGM[row][col] = num
        return outputOGM

class SelfNoiseFunction(object):
    """This class stores sets of bins collected over time. It numerically integrates over these sets of bins to attempt to find a function
    that represents the spatial distribution of self-noise generated by the sonar. This function can then be subtracted from the live data to
    eliminate self-noise. This does not remove noise due to watery randomness (suspended sediment, fish etc - this could be removed by
    short-term temporal integration of live data) or harmonics reflected from real objects which produce signatures beyond the object"""
    def __init__(self, maxSize=100, range=3):
        self.nbins = 400 # default value, but the class automatically detects if there is a different value, so no need to pass into constructor
        self.binsCollection = []
        self.maxSize = maxSize
        self.range = range
        self.result = None
    
    def add_bins_to_collection(self,bins):
        # Takes an array of bins and adds it to the collection
        # If the collection is already at max size, discard the first set in the collection
        # If this is the first array, set nbins to the length of bins (this assumes that nbins won't change during sonar operation)
        if self.binsCollection == []:
            self.nbins = len(bins)
        if len(self.binsCollection) >= self.maxSize:
            self.binsCollection.pop(0)
        self.binsCollection.append(bins)

    def integrate_collection(self):
        #rospy.loginfo(self.range)
        temp = np.zeros(self.nbins)
        for binArray in self.binsCollection:
            for i in range(self.nbins):
                intensityVal = binArray[i]
                temp[i] += intensityVal
        temp2 = []
        for i in range(self.nbins):
            rangeVal = round((float(i)/float(self.nbins))*float(self.range),2)
            temp2.append((rangeVal,temp[i]/len(self.binsCollection)))
        result = np.array(temp2,dtype=[("range",np.float32),("noiseIntensity",np.float32)])
        return result
    
    def smooth_data(self, data, averagingWindow):
        # A function that takes in the output of integrate_collection and smooths the noiseIntensity by averaging over a sliding window of a given size
        smoothedData = []
        for i in range(len(data)):
            rangeVal = data[i][0]
            windowMin = max(0, i - int(averagingWindow/2))
            windowMax = min(len(data)-1, i + int(averagingWindow/2))
            windowVals = [data[j][1] for j in range(windowMin, windowMax+1)]
            smoothedNoiseVal = np.mean(windowVals)            
            smoothedData.append((rangeVal, smoothedNoiseVal))
        return np.array(smoothedData, dtype=[("range",np.float32),("noiseIntensity",np.float32)])
    
    def show(self, data):
        # A blocking function that shows the self noise function
        #plt.close('all')
        y = data["noiseIntensity"]
        x = data["range"]
        plt.plot(x,y)
        plt.show()
    
    def save(self):
        data = self.integrate_collection()
        self.show(data) # show the unaverage function
        self.result = self.smooth_data(data, 5)
        np.savetxt("SelfNoiseFunction"+".csv", self.result, header="range,noiseIntensity")
        rospy.loginfo("Saved SelfNoiseFunction")
        self.show(self.result) # show the smoothed function

    def load(self):
        NoiseFuncDtype = np.dtype([("range", np.float32), ("noiseIntensity", np.float32)])
        data = np.loadtxt("SelfNoiseFunction.csv",dtype=NoiseFuncDtype, skiprows=1)
        self.result = data
        #self.show()
        
def callbackSave(data, OGM):
    rospy.loginfo("Saving with filename: {}".format(data.data))
    '''A function that saves the raw intensities with the filename contained in data'''
    '''Version with time
    time = str(rospy.get_rostime().secs)
    descriptiveName = str(data.data)
    filename = time + "-" + descriptiveName +".csv"
    OGM.save(filename)'''
    descriptiveName = str(data.data)
    filename = descriptiveName
    OGM.save(filename)

def callbackChatter(data,OGM):  
    OGM.update(data)
    if rospy.get_rostime().secs%2==0 and OGM.selfNoiseState != 'w':
        OGM.show()
        time = rospy.get_rostime().secs
        rospy.loginfo("Shown at time "+str(time))

def callbackHeading(data, OGM):
    if round(data.data,3) == round(data.data,2) and OGM.selfNoiseState != 'w':
        # only show the sonar image when the listener is not writing the self noise function
        # this is because the writing process shows a plot of the self noise function, which interferes with the OGM plot
        #rospy.loginfo("Shown")
        #OGM.show()
        pass
    else:
        #rospy.loginfo(data.data)
        pass

def listener():
    OGM = OccupancyGridMap(3,3, selfNoiseState='r')
    Noise = SelfNoiseFunction(100,3)
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('save', String, callbackSave, OGM)
    rospy.Subscriber('scan2', PointCloud2, callbackChatter, OGM)
    rospy.Subscriber('simpleHeading', Float32, callbackHeading, OGM)
    #rospy.Subscriber('binsList', Int32MultiArray, callbackBinsList, [Noise, OGM])
    rospy.loginfo("Spinning")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass