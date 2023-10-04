#!/usr/bin/env python3
#Librerias de ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
#Librerias de Python
import math

class JointAnglePositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_group_position_publisher')

        self.joint_final_angle = math.radians(85)
        self.joint_current_angle = 0.0

        self.angle_publisher = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)
         
        self.angle_msg = Float64MultiArray()
        self.interpolation_timer = self.create_timer(0.5, self.publish_interpolation_trajectory)
    
    def publish_interpolation_trajectory(self):
        self.joint_current_angle += math.radians(1)

        if(self.joint_current_angle >= self.joint_final_angle):
            self.joint_current_angle = self.joint_final_angle

        self.angle_msg.data = [
             self.joint_current_angle,
             self.joint_current_angle,
             self.joint_current_angle,
             self.joint_current_angle
        ]
        self.angle_publisher.publish(self.angle_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_angle_position_publisher_node = JointAnglePositionPublisher()
    try:
        rclpy.spin(joint_angle_position_publisher_node)
    except KeyboardInterrupt:
        joint_angle_position_publisher_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()