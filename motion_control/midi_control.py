import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ROSParameter, ParameterType
import rtmidi #use MIDIUtil library


class MidiController(Node):
    def __init__(self):
        super().__init__('midi_controller')

        # dictionary for servo button mapping
        self.servo_slider = {
            3: "angle"
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
        # Easy to change, simply change value for what preset pilot wants
        #comment any changes the the section next to value changeds
        # Note: Value cannot exceed 1.0.
        self.presets = {
            'slow': {
                'horizontal_sensitivity': 0.2, 
                'vertical_sensitivity': 0.2,
                'angular_sensitivity': 0.2,
                'slow_factor': 0.5 #keeping slow factor as a constant for these sensitivities.
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
        self.vector_client = self.create_client(SetParameters, '/vector_conversion/set_parameters') #creates a client for changing sensitivities in vector conversion
        self.servo_client = self.create_client(SetParameters, '/servo_controller/set_parameters') # creates a client also for updating the angle for servo control
        
        #warning message
        while not self.vector_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /vector_conversion/set_parameters service...')
       
        
        # MIDI setup
        self.midi = rtmidi.MidiIn() # opens input to the midi controller (check midiutil documentation)
        try:
            port = next(i for i, p in enumerate(self.midi.get_ports()) if "nanoKONTROL2" in p)
        except StopIteration:
            self.get_logger().error("nanoKONTROL2 MIDI device not found.")
            return
        self.midi.open_port(port) #opens port
        self.midi.set_callback(self.midi_callback)

    def apply_preset(self, preset_name):
        preset = self.presets.get(preset_name)
        if not preset:
            self.get_logger().warn(f"Preset '{preset_name}' not found.")
            return

        self.get_logger().info(f"Applying preset: {preset_name}") 
        for field, value in preset.items():
            self.set_parameter_on_target(field, value)

    def set_parameter_on_target(self, field, value, client, param_type=ParameterType.PARAMETER_DOUBLE): #added client
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
        future.add_done_callback(lambda f: self.get_logger().info(f"Result: {f.result()}")) #when callback finished

    
    #midi callback for manipulating msgs
    def midi_callback(self, message, timestamp):
        control_number = message[0][1] # message 0,1 is the number of the control, or button. USed in dictionary, for example control num 0 is slider 1.
        value = message[0][2] # message 0,2 is the value, 1 - 127, that the midi controller is giving for that button.

        # conditional logic that Checks for preset activation applies the preset
        if control_number in self.preset_buttons and value > 0:
            preset_name = self.preset_buttons[control_number]
            self.apply_preset(preset_name) # applies the preset only if teh control number is in the button mapping for presets
            return

        #handles sliders for sensitivity
        control_name = self.sensitivity_map.get(control_number, f"Unknown ({control_number})")
        # Rounds the values
        rounded = round(value / 127.0, 2) # divides the value by 127 to get the same value as motion control (so it is 0 - 1.0, even though midi is 0 - 127)

        # changes the sensitivity for sliders only if the control number is found in the dictionary for sensitivity
        if control_number in self.sensitivity_map:
            field = self.sensitivity_map[control_number]
            self.set_parameter_on_target(field, rounded, self.vector_client, param_type = ParameterType.PARAMETER_DOUBLE) #changed
        else:
            self.get_logger().info(f"{control_name} pressed (value: {value})") #logs the value and control name for ease of logging and debugging
    
        # servo motor conditional logic
        if control_number in self.servo_slider:
            field = self.servo_slider[control_number]
            #rounded is the value for midi from earlier, now it is being updated for angle on servo
            angle = int(120 + rounded * (240 - 120)) # change 0 - 1.0 from sensitivities to 120 - 240 for servo values
            self.set_parameter_on_target(field, angle, self.servo_client, param_type = ParameterType.PARAMETERDOUBLE)
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
