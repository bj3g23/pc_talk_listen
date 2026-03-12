#!/usr/bin/env python
# license removed for brevity
import rospy, math
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
import numpy as np
import random
 
class ScanSlice(object):

    """Scan slice.

    Attributes:
        bins: Array of intensities of each return.
        config: Sonar configuration at time of this slice.
        heading: Heading of sonar in radians.
        range: Range of scan in meters.
        timestamp: ROS timestamp.
    """

    def __init__(self, heading, bins):
        """Constructs ScanSlice instance.

        Args:
            heading: Heading of sonar in radians.
            bins: Array of intensities of each return.
            config: Sonar configuration at time of this slice.
        """
        self.heading = heading
        self.bins = bins
        self.nbins = 400
        #self.config = config
        self.range = 3
        self.timestamp = rospy.get_rostime()
    
    def to_PointCloud2(self, frame):
        """Returns a PointCloud2 message corresponding to scan slice.

        Args:
            frame: Frame ID.

        Returns:
            A sensor_msgs.msg.PointCloud2.
        """
        # Constructing the message and its header.
        header = Header()
        header.frame_id = frame
        header.stamp = self.timestamp

        # Defining the fields:
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # Setting up coordinate units
        r_step = float(self.range) / float(self.nbins)
        x_unit = math.cos(self.heading) * r_step
        y_unit = math.sin(self.heading) * r_step

        # Looping through bins and adding points to the cloud data
        points = []
        for r in range(1, self.nbins + 1):
            x = x_unit * r
            y = y_unit * r
            z = 0.00
            intensity = self.bins[r-1]
            points.append((x,y,z,intensity))
        
        outCloud = point_cloud2.create_cloud(header, fields, points)
        return outCloud

    def bins_message_generator(self, bins):
        dims = [MultiArrayDimension]
        dims[0].label, dims[0].size, dims[0].stride = "intensity", len(bins), len(bins)
        layout = MultiArrayLayout(dims, 0)
        outMsg = Int32MultiArray(layout, bins)
        return outMsg

def generate_bins(nbins):
        """Generates a set of bins according to some function for testing"""
        binsOut = []
        for i in range(nbins):
        #     if 0<i<10:
        #         binsOut.append(300)
        #     elif 10<i<30:
        #         binsOut.append(random.randint(200,300))
        #     elif 30<i<=45:
        #         binsOut.append(400)
        #     else:
        #         binsOut.append(0)
            binsOut.append(random.randint(0,100))
        return binsOut


def talker():
    pub = rospy.Publisher('scan2', PointCloud2, queue_size=10)
    binsPub = rospy.Publisher('binsList', Int32MultiArray, queue_size=1)
    headingPub = rospy.Publisher('simpleHeading', Float32, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        for head in range(1, int((np.pi*2)*100)):
            heading = float(head)/100
            bins = generate_bins(400)
            slice = ScanSlice(heading, bins)
            pc = slice.to_PointCloud2("sonar_frame")
            binsMsg = slice.bins_message_generator(bins)
            pub.publish(pc)
            headingPub.publish(heading)
            binsPub.publish(binsMsg)
            rate.sleep()
        
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass