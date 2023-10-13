import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

class ChaseObject(Node):
    '''
    '''
    def __init__(self):
        super().__init__('chase_object')

        self.target_angle = 0 # rad
        self.target_distance = 0.15 # meters
        self.ca = 1 # Angular Gain
        self.cl = 1 # Linear Gain

        self.move_subscriber = self.create_subscription(
            Float64MultiArray,
            '/obj_moveto',
            self.move_callback
        )

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)

        self.move_subscriber
        self.vel_publisher

    def move_callback(self, instructions):
        array = instructions.data
        distance = array[0]
        angle = array[1]
        self.p_controller(distance, angle)

    def p_controller(self, distance, angle):
        if abs(angle) > self.target_angle + 0.2618: # roughly +/- 15 degrees
            e = self.target_angle - angle
            kpa = (1 / abs(e) * (self.ca / (abs(e) ^ 2)))
            if e > 0: # right?
                ua = kpa * e
            else: # left?
                ua = -kpa * e
            self.publish_message(0.0, ua)
        elif abs(distance) > self.target_distance + 0.025: # +/- 2.5 cm
            e = self.target_distance - distance
            kpl = (1 / abs(e) * (self.cl / (abs(e) ^ 2)))
            if e > 0: # backwards
                ul = -kpl * e
            else: # forwards
                ul = kpl * e
            self.publish_message(ul, 0.0)
        else:
            self.publish_message(0.0, 0.0)

    def publish_message(self, distance, angle):
        msg = Twist()
        msg.linear.y = distance
        msg.angular.z = angle 
