import rclpy
from geometry_msgs.msg import Twist
import numpy as np
from numpy import sin, cos

wheel2center = 0.075
wheel_angles = [np.deg2rad(30), np.deg2rad(150), np.deg2rad(270)]

WHEEL_RADIUS = 0.04
JACOBIAN = np.array([[-sin(wheel_angles[0]), cos(wheel_angles[0]), wheel2center],
                     [-sin(wheel_angles[1]), cos(wheel_angles[1]), wheel2center],
                     [-sin(wheel_angles[2]), cos(wheel_angles[2]), wheel2center]])

class RobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__frontL_motor = self.__robot.getDevice('frontL_joint')
        self.__frontR_motor = self.__robot.getDevice('frontR_joint')
        self.__rear_motor = self.__robot.getDevice('rear_joint')

        self.__frontL_motor.setPosition(float('inf'))
        self.__frontL_motor.setVelocity(0)

        self.__frontR_motor.setPosition(float('inf'))
        self.__frontR_motor.setVelocity(0)

        self.__rear_motor.setPosition(float('inf'))
        self.__rear_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        lateral_speed = self.__target_twist.linear.y
        angular_speed = self.__target_twist.angular.z

        motion_matrix = (1/WHEEL_RADIUS) * JACOBIAN @ np.array([forward_speed, lateral_speed, angular_speed])
        command_motor_fRight = motion_matrix[0] # (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_fLeft = motion_matrix[1] # (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_rear = motion_matrix[2]

        # Print statements to output the calculated wheel velocities
        if np.linalg.norm(motion_matrix) > 0.01:
            print('\n\n---------')
            print(f"FR: {command_motor_fRight:.2f} rad/s")
            print(f"FL: {command_motor_fLeft:.2f} rad/s")
            print(f"R : {command_motor_rear:.2f} rad/s")

        self.__frontR_motor.setVelocity(command_motor_fRight)
        self.__frontL_motor.setVelocity(command_motor_fLeft)
        self.__rear_motor.setVelocity(command_motor_rear)