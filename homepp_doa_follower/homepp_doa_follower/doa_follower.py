import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32  # Import Int32
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import math
import time
import tf2_ros
from tf_transformations import euler_from_quaternion

class PID:
    def __init__(self, P=1.2, I=0.0, D=0.1):
        #1.0 0 0
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

    def update(self, feedback_value):
        if abs(self.SetPoint - feedback_value) > 3.14159:
            error = 6.28318 - abs(self.SetPoint - feedback_value)
        else:
            error =  abs(self.SetPoint - feedback_value)
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (delta_time > 0):
                self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

        return self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

class RotateToSoundSourceNode(Node):
    def __init__(self):
        super().__init__('rotate_to_sound_source_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sound_subscription = self.create_subscription(
            Int32,
            '/keyword_direction',
            self.sound_callback,
            10)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.rotation_complete_pub = self.create_publisher(String, 'rotation_complete', 10)  # New publisher for rotation complete
        self.sound_subscription  # prevent unused variable warning
        self.imu_subscription  # prevent unused variable warning
        self.pid = PID()
        self.sound_source_angle = 0.0
        self.current_orientation = 0.0
        self.rotation_started = False  # New flag for rotation started

        # Set the log level to debug
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().debug('doa_follower init')

    def sound_callback(self, msg):
        # Convert the sound source angle to a relative angle
        if msg.data > -90 and msg.data <= 180:
            msg.data = 90 - msg.data
        else:
            msg.data = - msg.data - 270    
        sound_source_angle_rad = np.radians(msg.data)
        self.get_logger().debug(f'Source data: {msg.data},sound rad: {sound_source_angle_rad}')
        self.sound_source_angle = self.current_orientation + sound_source_angle_rad
        # self.sound_source_angle = (self.sound_source_angle + np.pi) % (2 * np.pi) - np.pi
        if self.sound_source_angle > np.pi:
            self.sound_source_angle = self.sound_source_angle - 2 * np.pi
        elif self.sound_source_angle < -np.pi:
            self.sound_source_angle = self.sound_source_angle + 2 * np.pi
        self.pid.SetPoint = self.sound_source_angle
        self.rotation_started = True  # Set the flag to True when received sound source angle

        # Log the sound source angle
        # self.get_logger().debug(f'Sound source angle: {self.sound_source_angle}')

    def imu_callback(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.current_orientation = euler[2]  # yaw

        # Log the current orientation
        # self.get_logger().debug(f'Current orientation: {self.current_orientation},Sound source angle: {self.sound_source_angle}')

    def control_loop(self):
        rotation_speed = 0.0
        twist = Twist()
        if self.rotation_started:  # Only calculate and publish speed when rotation_started is True

            # Set your rotation speed
            rotation_speed = self.pid.update(self.current_orientation)
            rotation_speed_threshold = 0.001  # Set this to your desired threshold

            # Check if the rotation speed is below the threshold
            if abs(rotation_speed) < rotation_speed_threshold:
                rotation_speed = 0.0
                
            if abs(self.sound_source_angle - self.current_orientation) <= 0.01:
                self.rotation_started = False  # Set the flag to False when rotation is complete
                self.rotation_complete_pub.publish(String(data='Rotation complete'))  # Publish a message to the new topic
            if self.sound_source_angle < self.current_orientation:
                twist.angular.z = - rotation_speed
            else:
                twist.angular.z =  rotation_speed

            self.publisher_.publish(twist)
            self.get_logger().debug(f'speed: {twist.angular.z:.6f},Current orientation: {self.current_orientation:.6f},Sound source angle: {self.sound_source_angle:.6f}')

        rotation_speed = 0.0
        twist.angular.z = rotation_speed
        self.publisher_.publish(twist)
        # self.get_logger().debug('Done!')
        
        # Log the rotation speed

def main(args=None):
    rclpy.init(args=args)

    rotate_to_sound_source_node = RotateToSoundSourceNode()

    # Create a timer callback for the control loop
    timer_period = 0.1  # seconds
    rotate_to_sound_source_node.create_timer(timer_period, rotate_to_sound_source_node.control_loop)

    rclpy.spin(rotate_to_sound_source_node)

    rotate_to_sound_source_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
