#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import math

class ObjectRange(Node):
    '''
    
    '''
    def __init__(self):
        super().__init__('object_range')

        self.lidar_angle_offset = 0.0 # might need to change this to align camera and lidar
        self.fov = 62.2 * math.pi / 180 # rad
        self.img_width = 320 # px
        self.angle_per_pixel = self.fov / self.img_width
        self.scan_data = None

        lidar_qos_profile = QoSProfile(depth=5)
        lidar_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        lidar_qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        lidar_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile=lidar_qos_profile
        )

        self.loc_subscriber = self.create_subscription(
            Point,
            '/obj_location',
            self.loc_callback,
            qos_profile=5
        )

        self.move_publisher = self.create_publisher(Float64MultiArray, '/obj_moveto', 5)

        self.lidar_subscriber
        self.loc_subscriber
        self.move_publisher

    def lidar_callback(self, scan_data):
        self.scan_data = scan_data

    def loc_callback(self, obj_coords):
        angle = self.camera_cart2rad(obj_coords)
        dist = self.find_distance(angle, self.scan_data)
        if dist != -1:
            self.publish_message(dist, angle)
        else:
            self.publish_message(0.0, 0.0)

    def camera_cart2rad(self, coordinates):
        '''
        Converts cartesian coordinates to angle. 
        Assuming Raspberry Pi Cam 2, Pixels are Linear, and 0 rad is the center of the camera.
        Input: coordinates (geometry_msgs Point)
        Output: angle (rad)
        '''
        x = coordinates.x
        angle = (self.angle_per_pixel * x) - (self.fov / 2)
        return angle

    def find_distance(self, angle, scan_data):
        angle_min = scan_data.angle_min
        angle_max = scan_data.angle_max
        angle_inc = scan_data.angle_increment
        range_min = scan_data.range_min
        range_max = scan_data.range_max
        ranges = scan_data.ranges

        # angle of camera = angle_min + index * increment 
        if angle < 0: 
            angle += 2*math.pi
        try:
            index = (angle - angle_min) / angle_inc
            range = ranges[int(index) - 1]
            return range
        except:
            return -1

    def publish_message(self, dist, angle):
        msg = Float64MultiArray()
        msg.data = [dist, angle]
        self.move_publisher.publish(msg)
        #self.get_logger().info("Angle and Dist Message - Publishing: %s" %msg.data)

def main():
    rclpy.init()
    object_range = ObjectRange()
    try:
        rclpy.spin(object_range)
    except SystemExit:
        rclpy.get_logger("Object Range Node").info("Shutting Down")
    object_range.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 