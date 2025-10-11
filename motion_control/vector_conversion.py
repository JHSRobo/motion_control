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

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from std_srvs.srv import SetBool, Trigger

import math
from core.msg import Sensitivity


# This is the class that ROS2 spins up as a node
class VectorConverter(Node):

    def __init__(self):
        super().__init__('vector_conversion')
        
        self.log = self.get_logger() # Quick reference for logging

        # 30 hz loop to limit program speed, so we aren't crunching #s all the time.
        self.loop_rate = self.create_rate(30)

        self.thrusters_enabled = Bool()
        self.thrusters_enabled.data = False
        self.cached_input = False
        
        # Declare Publishers and Subscribers
        self.vector_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sensitivity_pub = self.create_publisher(Sensitivity, 'sensitivity', 10)
        self.thruster_status_pub = self.create_publisher(Bool, 'thruster_status', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Create a service for updating the camera feed with thruster status
        self.thruster_status_client = self.create_client(SetBool, 'thruster_status')
        # Create a service for publishing a sensitivity msg upon request
        self.first_sense_srv = self.create_service(Trigger, 'first_sensitivity', self.first_sense_callback)

        # Create a timer that checks for updated parameters 10x /second
        self.create_timer(0.1, self.update_parameters)

        # The following is used for handling a joystick bug
        self.initialized_axes = [False, False, False, False, False]

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
        self.slow_factor = 0.5
        self.inversion = False

        # Defines the settings that the GUI can actually control
        self.declare_parameter('horizontal_sensitivity', self.horizontal_sensitivity, sensitivity_descriptor)
        self.declare_parameter('vertical_sensitivity', self.vertical_sensitivity, sensitivity_descriptor)
        self.declare_parameter('angular_sensitivity', self.angular_sensitivity, sensitivity_descriptor)
        self.declare_parameter('slow_factor', self.slow_factor, sensitivity_descriptor)
        self.declare_parameter('inversion', self.inversion)

    # Publish our sensitivity for the first time
    # Almost the same as param_callback
    def first_sense_callback(self, request, response):
        self.horizontal_sensitivity = self.get_parameter('horizontal_sensitivity').value
        self.vertical_sensitivity = self.get_parameter('vertical_sensitivity').value
        self.angular_sensitivity = self.get_parameter('angular_sensitivity').value
        self.slow_factor = self.get_parameter('slow_factor').value
        self.inversion = self.get_parameter('inversion').value
        sense_msg = Sensitivity()
        sense_msg.horizontal = self.horizontal_sensitivity
        sense_msg.vertical = self.vertical_sensitivity
        sense_msg.angular = self.angular_sensitivity
        sense_msg.slow_factor = self.slow_factor
        self.sensitivity_pub.publish(sense_msg)
        return response


    # Update our parameters with the most recent settings from the GUI
    # The reason that this is not a parameter callback is because
    # That caused some VERY strange desync issues.
    def update_parameters(self):

        # Boolean for if the sensitivities have changed or not
        change = (self.horizontal_sensitivity != self.get_parameter('horizontal_sensitivity').value 
                  or self.vertical_sensitivity != self.get_parameter('vertical_sensitivity').value
                  or self.angular_sensitivity != self.get_parameter('angular_sensitivity').value
                  or self.slow_factor != self.get_parameter('slow_factor').value
                  or self.inversion != self.get_parameter('inversion').value
                  )

        # Update the values of our settings to reflect the parameters
        self.horizontal_sensitivity = self.get_parameter('horizontal_sensitivity').value
        self.vertical_sensitivity = self.get_parameter('vertical_sensitivity').value
        self.angular_sensitivity = self.get_parameter('angular_sensitivity').value
        self.slow_factor = self.get_parameter('slow_factor').value
        self.inversion = self.get_parameter('inversion').value
        
        # Populate a sensitivity message and publish it
        # Used by the camera viewer to show to the pilot
        if change:
            sense_msg = Sensitivity()
            sense_msg.horizontal = self.horizontal_sensitivity
            sense_msg.vertical = self.vertical_sensitivity
            sense_msg.angular = self.angular_sensitivity
            sense_msg.slow_factor = self.slow_factor
            self.sensitivity_pub.publish(sense_msg)


    def joy_callback(self, joy):

        # Enable or disable thrusters based on button press
        if joy.buttons[3] and not self.cached_input:
            self.thrusters_enabled.data = not self.thrusters_enabled.data
            if self.thrusters_enabled.data: self.log.info("Thrusters enabled")
            else: self.log.info("Thrusters disabled")

            # Update camera viewer with thruster status
            thruster_srv = SetBool.Request()
            thruster_srv.data = self.thrusters_enabled.data
            self.future = self.thruster_status_client.call_async(thruster_srv)

        self.cached_input = joy.buttons[3]

        # Enable or disable slow-mo
        # If slow-mo button is not pressed, set the scalar to 1.
        if joy.buttons[1]:
            slow_scale = self.slow_factor
        else:
            slow_scale = 1.0

        # The following code is for a bug with our joysticks.
        # Before the axes recieve any input, they broadcast a value of 1.0
        # even when in neutral position. So this code fixes that.
        for i in range(len(self.initialized_axes)):
            if not (joy.axes[i] == 1.0 or joy.axes[i] == -1.0):
                self.initialized_axes[i] = True

        # Create a twist message and populate it with joystick input
        # x is forwards, y is left, z is up.
        v = Twist()
        if self.initialized_axes[1]: v.linear.x = joy.axes[1]
        if self.initialized_axes[0]: v.linear.y = joy.axes[0]
        if self.initialized_axes[2]: v.linear.z = joy.axes[2]

        # Get roll effort from the controller triggers
        if self.initialized_axes[4]: v.angular.x = -joy.axes[4]
        # We skip angular.y because no pitch control... sadge...
        if self.initialized_axes[3]: v.angular.z = joy.axes[3]

        # Rotate linears by 105 degrees if inversion is active
        if self.inversion:
            theta = math.atan2(v.linear.y, v.linear.x)
            #self.log.info(str(theta))
            magnitude = math.hypot(v.linear.x, v.linear.y)
            #self.log.info(str(magnitude))
            theta -= math.radians(105)

            self.log.info("x: {}".format(math.cos(theta)))
            self.log.info("y: {}".format(math.sin(theta)))
            self.log.info(str(magnitude))

            v.linear.y = magnitude * math.sin(theta)
            v.linear.x = magnitude * math.cos(theta)

        v.linear.x *= (self.horizontal_sensitivity * slow_scale)
        v.linear.y *= (self.horizontal_sensitivity * slow_scale)
        v.linear.z *= (self.vertical_sensitivity * slow_scale)
        v.angular.x *= (self.angular_sensitivity * slow_scale)
        v.angular.z *= (self.angular_sensitivity * slow_scale)

        # If thrusters are off, tell bottomside to not control thrusters
        self.thruster_status_pub.publish(self.thrusters_enabled)

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
