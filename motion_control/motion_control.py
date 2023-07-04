# The purpose of this program is to subscribe to /cmd_vel (a topic which contains desired motion vectors)
# and turn that into thruster effort values.
# These effort values are then published and handled by other nodes.

# This program is also where sensetivity is adjusted, and thrusters are disabled.
# This is the node that the copilot controls in order to adjust the ROV's motion.

# I don't expect this program to hang around for too long
# It should be rewritten as a part of the motion control update for the 23-24 season
# Written by James Randall '24

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from core.msg import Float32Arr
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange


# This is the class that ROS2 spins up as a node
class MotionController(Node):

    def __init__(self):
        super().__init__('motion_control')

        # Quick reference for logging
        self.log = self.get_logger()

        self.thrusters_enabled = False
        self.cached_input = False
        
        # Declare Publishers and Subscribers
        self.vector_sub = self.create_subscription(Twist, 'cmd_vel', self.vector_callback, 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joy_sub # Just so it doesn't give that annoying "unused variable" warning
        self.vector_sub 
        self.effort_pub = self.create_publisher(Float32Arr, 'thrusters', 10)
        

        # Define parameters
        sensitivity_bounds = FloatingPointRange()
        sensitivity_bounds.from_value = 0.0
        sensitivity_bounds.to_value = 1.0
        sensitivity_bounds.step = 0.01
        sensitivity_descriptor = ParameterDescriptor(floating_point_range = [sensitivity_bounds])
        self.declare_parameter('lateral_sensitivity', 0.5, sensitivity_descriptor)
        self.declare_parameter('vertical_sensitivity', 0.5, sensitivity_descriptor)
        self.declare_parameter('angular_sensitivity', 0.5, sensitivity_descriptor)

    def vector_callback(self, v):

        lateral_sensitivity = self.get_parameter('lateral_sensitivity').value
        vertical_sensitivity = self.get_parameter('vertical_sensitivity').value
        angular_sensitivity = self.get_parameter('angular_sensitivity').value

        linearX = v.linear.x * lateral_sensitivity
        linearY = v.linear.y * vertical_sensitivity
        linearZ = v.linear.z * lateral_sensitivity
        angularZ = v.angular.z * angular_sensitivity

        # TEMPORARY: Turns vectors into thruster effort values.
        # Will be changed in advanced MoCo Rewrite
        thruster_vals = Float32Arr()
        
        if self.thrusters_enabled:
            thruster_vals.array = [
                linearX + linearZ + angularZ,
                -linearX + linearZ - angularZ,
                linearX - linearZ - angularZ,
                -linearX - linearZ + angularZ,
                linearY, 
                linearY ]
        else:
            thruster_vals.array = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        
        self.effort_pub.publish(thruster_vals)

    def joy_callback(self, joy):
        if joy.buttons[8] and not self.cached_input:
            self.thrusters_enabled = not self.thrusters_enabled
            if self.thrusters_enabled: self.log.info("Thrusters enabled")
            else: self.log.info("Thrusters disabled")
        self.cached_input = joy.buttons[8]

def main(args=None):
    rclpy.init(args=args)

    moco = MotionController()

    # Runs the program until shutdown is recieved
    rclpy.spin(moco)

    # On shutdown, kill node
    moco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
