import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ROSParameter, ParameterType
import rtmidi


class MidiController(Node):
    def __init__(self):
        super().__init__('midi_controller')

        # Slider mapping
        # When midi controller calls, sets parameter based on this dictionary
        self.sensitivity_map = {
            7: "horizontal_sensitivity",
            6: "vertical_sensitivity",
            5: "angular_sensitivity",
            4: "slow_factor"
        }

        # Preset definitions
        # Easy to change, simply change value for what preset pilot wants
        #comment any changes the the section next to value changeds
        self.presets = {
            'low': {
                'horizontal_sensitivity': 0.2, 
                'vertical_sensitivity': 0.2,
                'angular_sensitivity': 0.2,
                'slow_factor': 0.8
            },
            'medium': {
                'horizontal_sensitivity': 0.5,
                'vertical_sensitivity': 0.5,
                'angular_sensitivity': 0.5,
                'slow_factor': 0.5
            },
            'high': {
                'horizontal_sensitivity': 0.8,
                'vertical_sensitivity': 0.8,
                'angular_sensitivity': 0.8,
                'slow_factor': 0.2
            }
        }

        # Button mapping for presets
        # when specific button pressed, sets these presets
        self.preset_buttons = {
            39: 'low',
            55: 'medium',
            71: 'high'
        }

        # ROS parameter client setup
        self.param_client = self.create_client(SetParameters, '/motion_control/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /motion_control/set_parameters service...')

        # MIDI setup
        self.midi = rtmidi.MidiIn()
        try:
            port = next(i for i, p in enumerate(self.midi.get_ports()) if "nanoKONTROL2" in p)
        except StopIteration:
            self.get_logger().error("nanoKONTROL2 MIDI device not found.")
            return
        self.midi.open_port(port)
        self.midi.set_callback(self.midi_callback)

    def apply_preset(self, preset_name):
        preset = self.presets.get(preset_name)
        if not preset:
            self.get_logger().warn(f"Preset '{preset_name}' not found.")
            return

        self.get_logger().info(f"Applying preset: {preset_name}")
        for field, value in preset.items():
            self.set_parameter_on_target(field, value)

    def set_parameter_on_target(self, field, value):
        param = ROSParameter()
        param.name = field
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = value

        request = SetParameters.Request()
        request.parameters.append(param)

        future = self.param_client.call_async(request)
        self.get_logger().info(f"Requested parameter update: {field} = {value}")

        return future.result()

    #midi callback for manipulating msgs
    def midi_callback(self, message, timestamp):
        control_number = message[0][1]
        value = message[0][2]

        # Check for preset activation
        if control_number in self.preset_buttons and value > 0:
            preset_name = self.preset_buttons[control_number]
            self.apply_preset(preset_name)
            return

        # Handle sliders
        control_name = self.sensitivity_map.get(control_number, f"Unknown ({control_number})")
        rounded = round(value / 127.0, 2)

        if control_number in self.sensitivity_map:
            field = self.sensitivity_map[control_number]
            self.set_parameter_on_target(field, rounded)
        else:
            self.get_logger().info(f"{control_name} pressed (value: {value})")


def main(args=None):
    rclpy.init(args=args)
    node = MidiController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
