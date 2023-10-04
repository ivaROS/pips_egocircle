#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Vector3
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image, CameraInfo, LaserScan
import cv2
import numpy as np
from image_geometry import PinholeCameraModel


class SweptVolumePublisher():


    def scanCB(self, scan_msg):
        rospy.logdebug("Hallucinated scan callback")
        ranges = np.array(scan_msg.ranges)
        print type(ranges).__module__
        
        scan_size = len(ranges)
        scale = 1

        
        if scan_size > 1:
            marker = Marker()
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.header = scan_msg.header

            marker.scale = Vector3(x=scale,y=scale,z=scale)

            marker.ns="swept volume"

            marker.color = ColorRGBA(a=.25, b = 1)
            #Note: The following don't need to be changed
            # marker.pose
            # marker.id=0

            points = marker.points

            origin = [0,0,0]

            origin = Point(*origin)
            
            vals = np.logical_not(np.isnan(ranges))
            
            nvals = np.logical_and(vals[0:-1], vals[1:])
            angles = np.linspace(start=scan_msg.angle_min, stop=scan_msg.angle_max, num=scan_size, endpoint=False)
            scan_x = ranges*np.cos(angles)
            scan_y = ranges*np.sin(angles)
            
            def getPoint(index):
                return Point(x=scan_x[index],y=scan_y[index])
            
            for i in range(0,scan_size-1):
                if nvals[i]:
                    points += [getPoint(i), getPoint(i+1), origin]
            
            if nvals[-1]:
                points += [getPoint(-1), getPoint(0), origin]

            rospy.logdebug("Added points")

            self.marker_pub.publish(marker)
            rospy.logdebug("Publishing marker")


    def __init__(self):
        rospy.init_node('swept_volume_publisher')

        self.debug = False

        self.visualization_topic = 'swept_volume'

        self.scan_topic = 'scan_in'

        rospy.loginfo("Swept volume publisher starting up")

        self.camera_model = PinholeCameraModel()
        self.marker_pub = rospy.Publisher(self.visualization_topic, Marker, queue_size=5)

        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scanCB)

        rospy.loginfo("Hallucinated Robot Swept Volume Publisher ready")

        rospy.spin()


if __name__ == '__main__':
    try:
        SweptVolumePublisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")

