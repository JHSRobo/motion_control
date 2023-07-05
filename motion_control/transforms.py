# The purpose of this program is to subscribe to /cmd_vel (a topic which contains desired motion vectors)
# and turn that into transformations for the ROV.
# In ROS terms, this is the state_publisher program, but without the joint_state.
# These effort values are then published and handled by other nodes.

# Previously, we handled everything from joystick input to thruster effort values in one node.
# The reason we don't do that anymore is because it makes it difficult to monitor
# what is going on inside of the code, and also because shorter programs are easier
# to service.

# This code is loosely based off of the ROS2 Humble example state publisher
# https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html#

###################  O
##   IMPORTANT   ##
###################  O

# SOME ASPECTS OF THIS PROGRAM ARE FOR SIMULATION PURPOSES ONLY!
# This program broadcasts a transform, which is essentially a position in the world.
# We don't have the ability to measure our x and y positions in the world.
# z we can handle with depth sensor, and rotations can come from IMU
# (As a consequence, this means that depth hold and orientation lock are naturally
# built in to the ROV's thruster code)
# But because we can't measure x and y positions, we are approximating them here.
# On the actual ROV, our broadcasted x and y positions will be discarded in favor
# of a vector drive approach in those two degrees of freedom.
# So why are we broadcasting them to begin with?
# SIMULATION!!!!!


# Written by James Randall '24

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster, TransformStamped
from core_lib import quaternion_math, motion_simulator

class StatePublisher(Node):

    def __init__(self):
        super().__init__('motion_control')

        # Quick reference for logging
        self.log = self.get_logger()
        
        # Declare Publishers and Subscribers
        self.vector_sub = self.create_subscription(Twist, 'cmd_vel', self.vector_callback, 10)
        self.depth_sub = self.create_subscription(Float32, 'depth', self.depth_callback, 10)
        self.imu_sub = self.create_subscription(Quaternion, 'imu_data', self.imu_callback, 10)
        # A broadcaster is actually just a publisher, but ROS convention calls the
        # ROV state publisher a broadcaster.
        self.broadcaster = TransformBroadcaster(self, 10)

        # Simulators for x and y positions.
        # See message at the top.
        # This class was written by us, and can be found in the core pkg.
        self.xsim = motion_simulator.MotionSimulator(0)
        self.ysim = motion_simulator.MotionSimulator(0)

        # Variables for holding positions
        self.height = 0
        self.rotation = Quaternion()

        self.odom_trans = TransformStamped()


    def vector_callback(self, v):

        # Static world frame of referece - what the ROV moves in relation to
        # odom is short for odometry, or the static world frame.
        self.odom_trans.header.frame_id = 'base_link' 
        # ROV frame of reference - defined in our URDF file
        self.odom_trans.child_frame_id = 'base_link'
        
        now = self.get_clock().now() # Get current time

        # Generate a quaternion based on our commanded velocity
        # euler_to_quaternion was written by us, and can be found in the core pkg
        desired_rotation = quaternion_math.euler_to_quaternion(
            v.angular.x, v.angular.y, v.angular.z)

        # Update transforms (Temporarily just adding vectors)
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x += v.linear.x
        self.odom_trans.transform.translation.y += v.linear.y
        self.odom_trans.transform.translation.z += v.linear.z
        
        # quaternion_multiply was written by us, and can be found in the core pkg
        # It applies the second quaternion as a transformation to the first one.
        self.odom_trans.transform.rotation = quaternion_math.quaternion_multiply(
            self.odom_trans.transform.rotation, desired_rotation)

        # Broadcast them transformations
        self.broadcaster.sendTransform(self.odom_trans)


    def depth_callback(self, depth):
        self.height = 13 - depth.data

    def imu_callback(self, orientation):
        self.rotation = orientation.data


def main(args=None):
    rclpy.init(args=args)

    state_pub = StatePublisher()

    # Runs the program until shutdown is recieved
    rclpy.spin(state_pub)

    # On shutdown, kill node
    state_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
