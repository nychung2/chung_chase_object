#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

import numpy as np
import cv2
from cv_bridge import CvBridge
import sys


class DetectObject(Node):
    '''
    
    '''
    def __init__(self):
        super().__init__('detect_object')
        
        self.kernel = np.ones((5,5), np.uint16)

        self.color = [110, 120, 90] # HSV
        self.hue = 10
        self.sat = 50
        self.val = 50

        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(depth=5)
        image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 

        # make subcription to compressed image
        self.img_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed', # /image_raw/compressed <- real | sim -> /simulated_camera/image_raw/compressed
            self.callback,
            image_qos_profile)
        
        self.objloc_publisher = self.create_publisher(Point, '/obj_location', 5)

        self.img_subscriber
        self.objloc_publisher

    def callback(self, CompressedImage):
        imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        hsv_frame = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)
        low_bound = np.array([self.color[0]-self.hue, self.color[1]-self.sat, self.color[2]-self.val])
        high_bound = np.array([self.color[0]+self.hue, self.color[1]+self.sat, self.color[2]+self.val])
        hsv_threshold = cv2.inRange(hsv_frame, low_bound, high_bound)
        close_dialation = cv2.dilate(hsv_threshold, self.kernel, iterations=1)
        close_erosion = cv2.erode(close_dialation, self.kernel, iterations=1)
        open_erosion = cv2.erode(close_erosion, self.kernel, iterations=1)
        open_dialation = cv2.dilate(open_erosion, self.kernel, iterations=1)
        contours, hierarchy = cv2.findContours(open_dialation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            x, y, w, h = self.get_object_location(contours)
            cx, cy = self.get_center(x,y,w,h)
            self.publish_message(cx,cy)
        else:
            self.publish_message(160.0,0.0)


    def publish_message(self, x, y):
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0
        self.objloc_publisher.publish(msg)
        #self.get_logger().info("Location of Object - Publishing: %s, %s, %s" %(msg.x, msg.y, msg.z))

    def get_object_location(self, contours):
        if len(contours) != 0:
            c = max(contours, key = cv2.contourArea)
            if cv2.contourArea(c) < 100:
                return 160, 120, 0 ,0 # aka dont move
            x,y,w,h = cv2.boundingRect(c)
            return x, y, w ,h
        else:
            return None, None, None, None

    def get_center(self, x,y,w,h):
        cx = x + w/2
        cy = y + h/2
        print("Center Coordinates of Object = (%s,%s)" %(cx, cy))
        return (cx, cy)

def main():
    rclpy.init()
    detect_object = DetectObject()
    try:
        rclpy.spin(detect_object)
    except SystemExit:
        rclpy.get_logger("Detect Object Node").info("Shutting Down")
    detect_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
