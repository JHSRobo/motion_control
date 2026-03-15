import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ROSParameter, ParameterType
import rtmidi  # use MIDIUtil library


class MidiController(Node):
    def __init__(self):
        super().__init__('midi_controller')

        # dictionary for servo button mapping
        self.servo_slider = {
            3: "angle"
        }

        self.brushed_motor = {
            #change to what slide r is needed
            2: "brushed_motor_speed"
        }
        
        # Slider mapping
        # When midi controller calls, sets parameter based on this dictionary
        self.sensitivity_map = {
            7: "horizontal_sensitivity",
            6: "vertical_sensitivity",
            5: "angular_sensitivity",
            4: "slow_factor"
        }

        # Preset definitions
        # Easy to change, just change value for what preset pilot wants
        
        self.presets = {
            'slow': {
                'horizontal_sensitivity': 0.2,
                'vertical_sensitivity': 0.2,
                'angular_sensitivity': 0.2,
                'slow_factor': 0.5  # keeping slow factor as a constant for these sensitivities.
            },
            'medium': {
                'horizontal_sensitivity': 0.5,
                'vertical_sensitivity': 0.5,
                'angular_sensitivity': 0.5,
                'slow_factor': 0.5
            },
            'rapid': {
                'horizontal_sensitivity': 1.0,
                'vertical_sensitivity': 1.0,
                'angular_sensitivity': 1.0,
                'slow_factor': 0.5
            }
        }

        # Button mapping for presets
        # when specific button pressed, sets these presets. values above, button mapping below.
        self.preset_buttons = {
            39: 'slow',
            55: 'medium',
            71: 'rapid'
        }

        # ROS clients
        self.vector_client = self.create_client(SetParameters, '/vector_conversion/set_parameters')
        self.servo_client = self.create_client(SetParameters, '/servo_controller/set_parameters')
        self.brushed_motor_client = self.create_client(SetParameters, '/brushed_motor/set_parameters')

        #these were debugging so I could tell if they werent working, commented out now but kept in jsut in case
        
        # Wait for vector_conversion
        #while not self.vector_client.wait_for_service(timeout_sec=1.0):
            #self.get_logger().warn('Waiting for /vector_conversion/set_parameters service...')

        # Wait for servo_controller so servo updates don't silently fail
        #while not self.servo_client.wait_for_service(timeout_sec=1.0):
            #self.get_logger().warn('Waiting for /servo_controller/set_parameters service...')

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
            # FIX: pass the client
            self.set_parameter_on_target(field, value, self.vector_client)

    def set_parameter_on_target(self, field, value, client,
                                param_type=ParameterType.PARAMETER_DOUBLE):
        param = ROSParameter()
        param.name = field
        param.value.type = param_type

        if param_type == ParameterType.PARAMETER_DOUBLE:
            param.value.double_value = float(value)
        elif param_type == ParameterType.PARAMETER_INTEGER:
            param.value.integer_value = int(value)

        request = SetParameters.Request()
        request.parameters.append(param)

        future = client.call_async(request)
        self.get_logger().info(f"Requested parameter update: {field} = {value}")
        future.add_done_callback(
            lambda f: self.get_logger().info(f"Result for {field}: {f.result()}")
        )

    # midi callback for manipulating msgs
    def midi_callback(self, message, timestamp):
        # Safety: make sure message has expected structure
        if not message or len(message[0]) < 3:
            return

        control_number = message[0][1]  # which control (slider/button)
        value = message[0][2]           # 0–127

        # Check for preset activation
        if control_number in self.preset_buttons and value > 0:
            preset_name = self.preset_buttons[control_number]
            self.apply_preset(preset_name)
            return

        # Compute normalized value once (0.0–1.0)
        rounded = round(value / 127.0, 2)

        # Sensitivity sliders
        if control_number in self.sensitivity_map:
            field = self.sensitivity_map[control_number]
            self.set_parameter_on_target(
                field,
                rounded,
                self.vector_client,
                param_type=ParameterType.PARAMETER_DOUBLE
            )
            return

        # Servo motor conditional logic
        if control_number in self.servo_slider:
            field = self.servo_slider[control_number]
            # map 0–1.0 → 120–240 for servo values
            angle = int(120 + rounded * (240 - 120))
            self.set_parameter_on_target(
                field,
                angle,
                self.servo_client,
                param_type=ParameterType.PARAMETER_INTEGER  # changes to integer type not double, otherwise wouldn't work
            )
            return

        # Brushed motor conditional logic
        if control_number in self.brushed_motor:
            field = self.brushed_motor[control_number]
            # brushed motor expects a double 0.0–1.0
            self.set_parameter_on_target(
                field,
                rounded,
                self.brushed_motor_client,
                param_type=ParameterType.PARAMETER_DOUBLE
            )
            return

        #  logging for unknown controls
        self.get_logger().info(f"Unknown control {control_number} (value: {value})")


def main(args=None):
    rclpy.init(args=args)
    node = MidiController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
