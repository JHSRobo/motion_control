# This program's purpose is to turn joystick input into a vector.
# There is a lot of code out there to do this already.
# So the reason that we wrote this ourselves was:
  # We have a unique method of controlling the ROV's roll
  # This is the best step for implementing things like sensitivity / thruster status

# This is the "First Step" in our motion control software.
# We publish a twist msg with a linear and rotational vector
# The conversion into the transform frame is done by transforms.py

# Written by James Randall '24

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange


# This is the class that ROS2 spins up as a node
class VectorConverter(Node):

    def __init__(self):
        super().__init__('motion_control')
        
        self.log = self.get_logger() # Quick reference for logging

        # 30 hz loop to limit program speed, so we aren't crunching #s all the time.
        self.loop_rate = self.create_rate(30)

        self.thrusters_enabled = False
        self.cached_input = False
        
        # Declare Publishers and Subscribers
        self.vector_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Define parameters

        # Next couple lines are for creating a "parameter descriptiong"
        # which sets the range on the sliders in the rqt gui
        sensitivity_bounds = FloatingPointRange()
        sensitivity_bounds.from_value = 0.0
        sensitivity_bounds.to_value = 1.0
        sensitivity_bounds.step = 0.01
        sensitivity_descriptor = ParameterDescriptor(floating_point_range = [sensitivity_bounds])

        # Defines the settings that the GUI can actually control
        self.declare_parameter('lateral_sensitivity', 0.5, sensitivity_descriptor)
        self.declare_parameter('vertical_sensitivity', 0.5, sensitivity_descriptor)
        self.declare_parameter('angular_sensitivity', 0.5, sensitivity_descriptor)
        self.declare_parameter('roll_control', False)


    def joy_callback(self, joy):

        # Enable or disable thrusters based on button press
        if joy.buttons[8] and not self.cached_input:
            self.thrusters_enabled = not self.thrusters_enabled
            if self.thrusters_enabled: self.log.info("Thrusters enabled")
            else: self.log.info("Thrusters disabled")
        self.cached_input = joy.buttons[8]

        # Update our parameters with the most recent settings from the GUI
        lateral_sensitivity = self.get_parameter('lateral_sensitivity').value
        vertical_sensitivity = self.get_parameter('vertical_sensitivity').value
        angular_sensitivity = self.get_parameter('angular_sensitivity').value
        roll_enabled = self.get_parameter('roll_control').value

        # Create a twist message and populate it with joystick input
        # x is forwards, y is left, z is up.
        v = Twist()
        v.linear.x = joy.axes[1]
        v.linear.y = joy.axes[0]
        v.linear.z = joy.axes[4]
        
        # Here is our custom roll implementation: We use the triggers
        roll = (joy.axes[2] - joy.axes[5]) / 2
        if roll_enabled: v.angular.x = roll

        # We skip angular.y because no pitch control... sadge...
        v.angular.z = joy.axes[3]

        # Scale effort values based on sensitivity
        v.linear.x *= lateral_sensitivity
        v.linear.y *= lateral_sensitivity
        v.linear.z *= vertical_sensitivity
        v.angular.x *= angular_sensitivity
        v.angular.z *= angular_sensitivity

        # If thrusters are off, wipe the vector.
        if not self.thrusters_enabled:
            v = Twist()

        # Publish our vector
        self.vector_pub.publish(v)

        # Sleep a variable amt of time so this callback runs 30 times / second
        #self.loop_rate.sleep() 


def main(args=None):
    rclpy.init(args=args)

    vectorCon = VectorConverter()

    # Runs the program until shutdown is recieved
    rclpy.spin(vectorCon)

    # On shutdown, kill node
    vectorCon.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
