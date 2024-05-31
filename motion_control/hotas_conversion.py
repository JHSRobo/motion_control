import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

class XboxToHotasNode:
    def __init__(self):
        self.node = rclpy.create_node('xbox_to_hotas_node')
        self.subscriber = self.node.create_subscription(Joy, '/xbox_joy', self.joy_callback, 10)
        self.publisher = self.node.create_publisher(Joy, '/joy', 10)
        
        # Mapping from Xbox controller axes/buttons to Thrustmaster T Flight Hotas X axes/buttons
        self.axes_mapping = [0, 1, 4, 3]  # Map axes 0, 1, 2, 5 from Xbox to Hotas X
        self.buttons_mapping = [5, 4, 1, 8]  # Map buttons 0-9 from Xbox to Hotas X

    def sanitize(self, value):
        if abs(value) < 0.05: value = 0.0
        return round(value, 3)

    def joy_callback(self, msg):
        hotas_msg = Joy()
        
        # Map axes
        for index in self.axes_mapping:
            hotas_msg.axes.append(self.sanitize(msg.axes[index]))

        # Append the roll axis (different because it uses triggers)
        hotas_msg.axes.append((-msg.axes[5] + msg.axes[2]) / 2)
            
        # Map buttons
        for index in self.buttons_mapping:
            hotas_msg.buttons.append(msg.buttons[index])

        hotas_msg.buttons.extend([0, 0, 0, 0])

        # Map D-Pad Axes to buttons
        if msg.axes[7] == 1.0: hotas_msg.buttons[4] = 1
        if msg.axes[7] == -1.0: hotas_msg.buttons[6] = 1
        if msg.axes[6] == -1.0: hotas_msg.buttons[5] = 1
        if msg.axes[6] == 1.0: hotas_msg.buttons[7] = 1

            
        # Publish the modified Joy message
        self.publisher.publish(hotas_msg)

def main(args=None):
    rclpy.init(args=args)
    xbox_to_hotas_node = XboxToHotasNode()
    rclpy.spin(xbox_to_hotas_node.node)
    xbox_to_hotas_node.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
