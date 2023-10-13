import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import math

class ObjectRange(Node):
    '''
    
    '''
    def __init__(self):
        super().__init__('object_range')

        self.lidar_angle_offset = 0.0 # might need to change this to align camera and lidar
        self.fov = 62.2 * math.pi / 180 # rad
        self.img_width = 640 # px
        self.angle_per_pixel = self.fov / self.img_width
        self.scan_data = None

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback
        )

        self.loc_subscriber = self.create_subscription(
            Point,
            '/obj_location',
            self.loc_callback
        )

        self.move_publisher = self.create_publisher(Float64MultiArray, '/obj_moveto', 5)

        self.lidar_subscriber
        self.loc_subscriber
        self.move_publisher

    def lidar_callback(self, scan_data):
        self.scan_data = scan_data

    def loc_callback(self, obj_coords):
        angle_min = self.scan_data.angle_min + self.lidar_angle_offset
        angle_max = self.scan_data.angle_max + self.lidar_angle_offset
        angle = self.camera_cart2rad(obj_coords)
        if angle < angle_max and angle > angle_min: 
            dist = self.find_distance(angle, self.scan_data)
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
        angle = (self.angle_per_pixel * (x + 1)) - (self.fov / 2)
        return angle

    def find_distance(angle, scan_data):
        range_min = scan_data.range_min
        range_max = scan_data.range_max
        ranges = scan_data.ranges
        running_count = 0
        counter = 0
        for range in ranges:
            if not(range < range_min or range > range_max):
                running_count += range
                counter += 1
        if counter > 0:
            return running_count / counter
        else:
            return -1.0 # Fail

    def publish_message(self, dist, angle):
        msg = Float64MultiArray
        msg.data = [dist, angle]
        self.move_publisher.publish(msg)

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