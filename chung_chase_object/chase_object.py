#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time

class ChaseObject(Node):
    '''
    '''
    def __init__(self):
        super().__init__('chase_object')

        self.target_angle = 0.0 # rad -> middle
        self.target_distance = 0.5 # meters
        self.ca = 1.2# Angular Gain
        self.cl = 0.5 # Linear Gain

        self.move_subscriber = self.create_subscription(
            Float64MultiArray,
            '/obj_moveto',
            self.move_callback,
            qos_profile=5
        )

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)

        self.move_subscriber
        self.vel_publisher

    def move_callback(self, instructions):
        #self.get_logger().info("Time: %s" %(time.time()))
        array = instructions.data
        distance = array[0]
        angle = array[1]
        self.p_controller(distance, angle)

    def p_controller(self, distance, angle):
        #self.get_logger().info("angle " + str(angle))
        #self.get_logger().info("distance " + str(distance))
        e = self.target_angle - angle
        if abs(e) > 0.0873: # roughly +/- 5 degrees
            kpa = 4
            ua = kpa * e
            if ua > 2.0:
                ua = 2.0
            if ua < -2.0:
                ua = -2.0
        else:
            ua = 0.0
        e = self.target_distance - distance
        if abs(e) > 0.08: # +/- 0.05m or 8cm 
            kpl = -0.8
            ul = kpl * e
            if ul > 0.15:
                ul = 0.15
            if ul < - 0.15:
                ul = -0.15
        else:
            ul = 0.0
        self.publish_message(ul, ua)

    def publish_message(self, distance, angle):
        msg = Twist()
        msg.linear.x = distance
        msg.angular.z = angle 
        self.vel_publisher.publish(msg)
        #self.get_logger().info("Twist Message - Publishing: %s, %s" %(msg.linear.x, msg.angular.z))

def main():
    rclpy.init()
    chase_object = ChaseObject()
    try:
        rclpy.spin(chase_object)
    except SystemExit:
        rclpy.get_logger("Chase Object Node").info("Shutting Down")
    chase_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
