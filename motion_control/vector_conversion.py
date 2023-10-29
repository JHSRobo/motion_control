# This program's purpose is to turn joystick input into a vector.
# There is a lot of code out there to do this already.
# So the reason that we wrote this ourselves was:
  # We have a unique method of controlling the ROV's roll
  # This is the best step for implementing things like sensitivity / thruster status

# This is the "First Step" in our motion control software.
# We publish a twist msg with a linear and rotational vector.

# Written by James Randall '24

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from std_srvs.srv import SetBool, Trigger

from core.msg import Sensitivity
import time


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
        self.sensitivity_pub = self.create_publisher(Sensitivity, 'sensitivity', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Create a service for updating the camera feed with thruster status
        self.thruster_status_client = self.create_client(SetBool, 'thruster_status')
        # Create a service for publishing a sensitivity msg upon request
        self.first_sense_srv = self.create_service(Trigger, 'first_sensitivity', self.first_sense_callback)

        # Create a timer that checks for updated parameters 5x /second
        self.create_timer(0.2, self.update_parameters)

        # Define parameters

        # Next couple lines are for creating a "parameter descriptiong"
        # which sets the range on the sliders in the rqt gui
        sensitivity_bounds = FloatingPointRange()
        sensitivity_bounds.from_value = 0.0
        sensitivity_bounds.to_value = 1.0
        sensitivity_bounds.step = 0.01
        sensitivity_descriptor = ParameterDescriptor(floating_point_range = [sensitivity_bounds])

        # Define the initial values for each sense
        self.horizontal_sensitivity = 0.5
        self.vertical_sensitivity = 0.5
        self.angular_sensitivity = 0.3

        # Defines the settings that the GUI can actually control
        self.declare_parameter('horizontal_sensitivity', self.horizontal_sensitivity, sensitivity_descriptor)
        self.declare_parameter('vertical_sensitivity', self.vertical_sensitivity, sensitivity_descriptor)
        self.declare_parameter('angular_sensitivity', self.angular_sensitivity, sensitivity_descriptor)

    # Publish our sensitivity for the first time
    # Almost the same as param_callback
    def first_sense_callback(self, request, response):
        self.horizontal_sensitivity = self.get_parameter('horizontal_sensitivity').value
        self.vertical_sensitivity = self.get_parameter('vertical_sensitivity').value
        self.angular_sensitivity = self.get_parameter('angular_sensitivity').value
        sense_msg = Sensitivity()
        sense_msg.horizontal = self.horizontal_sensitivity
        sense_msg.vertical = self.vertical_sensitivity
        sense_msg.angular = self.angular_sensitivity
        self.sensitivity_pub.publish(sense_msg)
        return response


    # Update our parameters with the most recent settings from the GUI
    # The reason that this is not a parameter callback is because
    # That caused some VERY strange desync issues.
    def update_parameters(self):

        # Boolean for if the sensitivities have changed or not
        change = (self.horizontal_sensitivity != self.get_parameter('horizontal_sensitivity').value 
                  or self.vertical_sensitivity != self.get_parameter('vertical_sensitivity').value
                  or self.angular_sensitivity != self.get_parameter('angular_sensitivity').value)

        # Update the values of our settings to reflect the parameters
        self.horizontal_sensitivity = self.get_parameter('horizontal_sensitivity').value
        self.vertical_sensitivity = self.get_parameter('vertical_sensitivity').value
        self.angular_sensitivity = self.get_parameter('angular_sensitivity').value
        
        # Populate a sensitivity message and publish it
        # Used by the camera viewer to show to the pilot
        if change:
            sense_msg = Sensitivity()
            sense_msg.horizontal = self.horizontal_sensitivity
            sense_msg.vertical = self.vertical_sensitivity
            sense_msg.angular = self.angular_sensitivity
            self.sensitivity_pub.publish(sense_msg)


    def joy_callback(self, joy):

        # Enable or disable thrusters based on button press
        if joy.buttons[8] and not self.cached_input:
            self.thrusters_enabled = not self.thrusters_enabled
            if self.thrusters_enabled: self.log.info("Thrusters enabled")
            else: self.log.info("Thrusters disabled")

            # Update camera viewer with thruster status
            thruster_srv = SetBool.Request()
            thruster_srv.data = self.thrusters_enabled
            self.future = self.thruster_status_client.call_async(thruster_srv)

        self.cached_input = joy.buttons[8]

        # Create a twist message and populate it with joystick input
        # x is forwards, y is left, z is up.
        v = Twist()
        v.linear.x = joy.axes[1]
        v.linear.y = joy.axes[0]
        v.linear.z = joy.axes[4]

        # Get roll effort from the controller triggers
        v.angular.x = (joy.axes[2] - joy.axes[5]) / 2
        # We skip angular.y because no pitch control... sadge...
        v.angular.z = joy.axes[3]

        # Scale effort values based on sensitivity
        v.linear.x *= self.horizontal_sensitivity
        v.linear.y *= self.horizontal_sensitivity
        v.linear.z *= self.vertical_sensitivity
        v.angular.x *= self.angular_sensitivity
        v.angular.z *= self.angular_sensitivity

        # If thrusters are off, wipe the vector.
        if not self.thrusters_enabled:
            v = Twist()

        # Publish our vector
        self.vector_pub.publish(v)


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
