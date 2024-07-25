import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import math
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Initialize serial communication with the Pololu Micro Maestro on Raspberry Pi 5.
        # /dev/ttyAMA0 is the default UART device on the Raspberry Pi 5.
        # The baudrate is set to 9600, and timeout is set to 1 second for reading responses.
        self.serial_port = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('Robot Controller Node Initialized')

        # Do not set any initial positions for the servos
        self.last_base_angle = None
        self.last_base_position = None  # Do not initialize to 6000.
        self.link1_position = None  # Do not initialize to 6000.
        self.link2_position = None  # Do not initialize to 6000.
        self.gripper_position = None  # Do not initialize to 6000.
        self.servo4_position = None  # Do not initialize to 6000.
        self.last_gripper_angle = None
        self.last_gripper_position = None  # Do not initialize to 6000.
        self.last_command_time = time.time()
        
        # Dictionary to track the time each button was pressed:
        self.button_press_times = {
            'btn_a': None,  # Time when button A was pressed.
            'btn_b': None,  # Time when button B was pressed.
            'btn_x': None,  # Time when button X was pressed.
            'btn_y': None,  # Time when button Y was pressed.
            'btn_lb': None,  # Time when button LB was pressed.
            'btn_rb': None  # Time when button RB was pressed.
        }
        
        self.initial_ramp_speed = 80
        self.max_ramp_speed = 320  # Maximum ramp speed.
        self.ramp_rate = 2  # Rate at which the ramp speed increases.

    def send_command(self, cmd):
        self.serial_port.write(cmd)
        self.get_logger().info(f'Sent command: {cmd}')

    def set_servo_target(self, channel, target):
        cmd = bytearray([0xAA, 0x0C, 0x04, channel, int(target) & 0x7F, (int(target) >> 7) & 0x7F])
        self.send_command(cmd)

    def ramp_value(self, initial_value, base_increment, elapsed_time):
        """Calculate ramped value based on elapsed time with gradually increasing speed."""
        # Calculate a normalized time factor between 0 and 1:
        ramp_factor = min(elapsed_time / self.ramp_rate, 1)
        
        # Calculate the ramp speed, which increases quadratically over time:
        ramp_speed = base_increment + (ramp_factor ** 2) * (self.max_ramp_speed - base_increment)
        
        # Calculate the new value by adding the ramp speed to the initial value:
        ramped_value = initial_value + ramp_speed
        
        return ramped_value

    def joy_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_command_time < 0.01:  # Limit command frequency to 100 Hz.
            return
        self.last_command_time = current_time

        x = msg.axes[0]  # Axis 0
        y = msg.axes[1]  # Axis 1
        z = msg.axes[2]  # Axis 2
        rz = msg.axes[3]  # Axis 3
        btn_a = msg.buttons[0]  # Button A
        btn_b = msg.buttons[1]  # Button B
        btn_x = msg.buttons[3]  # Button X
        btn_y = msg.buttons[4]  # Button Y
        btn_lb = msg.buttons[6]  # Button LB
        btn_rb = msg.buttons[7]  # Button RB

        # Handle base rotation (Channel 0):
        self.handle_base_rotation(x, y)

        # Handle link 1 (Channel 1):
        if btn_a:  # BtnA pressed.
            if self.link1_position is None:
                self.link1_position = self.get_current_servo_position(1)  # Start from the current position.
            if self.button_press_times['btn_a'] is None:
                self.button_press_times['btn_a'] = current_time
            elapsed_time = current_time - self.button_press_times['btn_a']
            increment = self.ramp_value(0, self.initial_ramp_speed, elapsed_time)
            self.link1_position = min(self.link1_position + increment, 8000)
        elif btn_b:  # BtnB pressed.
            if self.link1_position is None:
                self.link1_position = self.get_current_servo_position(1)  # Start from the current position.
            if self.button_press_times['btn_b'] is None:
                self.button_press_times['btn_b'] = current_time
            elapsed_time = current_time - self.button_press_times['btn_b']
            decrement = self.ramp_value(0, self.initial_ramp_speed, elapsed_time)
            self.link1_position = max(self.link1_position - decrement, 4000)
        else:
            self.button_press_times['btn_a'] = None
            self.button_press_times['btn_b'] = None

        if btn_a or btn_b:  # Only set target if there's a change.
            self.set_servo_target(1, self.link1_position)

        # Handle link 2 (Channel 2)
        if btn_x:  # BtnX pressed.
            if self.link2_position is None:
                self.link2_position = self.get_current_servo_position(2)  # Start from the current position.
            if self.button_press_times['btn_x'] is None:
                self.button_press_times['btn_x'] = current_time
            elapsed_time = current_time - self.button_press_times['btn_x']
            increment = self.ramp_value(0, self.initial_ramp_speed, elapsed_time)
            self.link2_position = min(self.link2_position + increment, 8000)
        elif btn_y:  # BtnY pressed.
            if self.link2_position is None:
                self.link2_position = self.get_current_servo_position(2)  # Start from the current position.
            if self.button_press_times['btn_y'] is None:
                self.button_press_times['btn_y'] = current_time
            elapsed_time = current_time - self.button_press_times['btn_y']
            decrement = self.ramp_value(0, self.initial_ramp_speed, elapsed_time)
            self.link2_position = max(self.link2_position - decrement, 4000)
        else:
            self.button_press_times['btn_x'] = None
            self.button_press_times['btn_y'] = None

        if btn_x or btn_y:  # Only set target if there's a change.
            self.set_servo_target(2, self.link2_position)

        # Handle gripper (Channel 4) using LB and RB buttons:
        if btn_lb:  # BtnLB pressed.
            if self.servo4_position is None:
                self.servo4_position = self.get_current_servo_position(4)  # Start from the current position.
            self.servo4_position = min(self.servo4_position + 480, 8000)
        elif btn_rb:  # BtnRB pressed.
            if self.servo4_position is None:
                self.servo4_position = self.get_current_servo_position(4)  # Start from the current position.
            self.servo4_position = max(self.servo4_position - 480, 4000)
        else:
            self.button_press_times['btn_lb'] = None
            self.button_press_times['btn_rb'] = None

        if btn_lb or btn_rb:  # Only set target if there's a change.
            self.set_servo_target(4, self.servo4_position)

        # Handle wrist servo (Channel 3) using Axis 2 (z) and Axis 3 (rz):
        self.handle_gripper_rotation(z, rz)

    def handle_base_rotation(self, x, y):
        angle = math.atan2(y, x)
        angle_degrees = math.degrees(angle)
        magnitude = math.sqrt(y ** 2 + x ** 2)

        if magnitude > 0.05:
            if self.last_base_angle is None:
                self.last_base_angle = angle_degrees
                return

            angle_diff = angle_degrees - self.last_base_angle
            if math.fabs(angle_diff) > 2:
                direction = 1 if angle_diff > 0 else -1  # If angle_diff > 0 then clockwise, else counterclockwise.
            else:
                angle_degrees = self.last_base_angle
                direction = self.last_direction
        else:
            direction = 0

        if self.last_base_position is None:
            self.last_base_position = self.get_current_servo_position(0)  # Start from the current position.
        position_change = direction * 48  # Increased increment to speed up rotation.
        position = self.last_base_position + position_change
        position = max(min(position, 8000), 4000)  # Make sure the position is within the full range.
        if direction != 0:
            self.set_servo_target(0, position)
        self.last_base_position = position
        self.last_base_angle = angle_degrees
        self.last_direction = direction

    def handle_gripper_rotation(self, z, rz):
        angle = math.atan2(rz, z)
        angle_degrees = math.degrees(angle)
        magnitude = math.sqrt(rz ** 2 + z ** 2)

        if magnitude > 0.05:
            if self.last_gripper_angle is None:
                self.last_gripper_angle = angle_degrees
                return

            angle_diff = angle_degrees - self.last_gripper_angle
            if math.fabs(angle_diff) > 2:
                direction = -1 if angle_diff > 0 else 1  # In this case, counterclockwise if greater than zero, else clockwise.
            else:
                angle_degrees = self.last_gripper_angle
                direction = self.last_direction
        else:
            direction = 0

        if self.last_gripper_position is None:
            self.last_gripper_position = self.get_current_servo_position(3)  # Start from the current position.
        position_change = direction * 64  # Increased step size increment to speed up rotation.
        position = self.last_gripper_position + position_change
        position = max(min(position, 8000), 4000)  # Make sure the position is within the full range.
        if direction != 0:
            self.set_servo_target(3, position)
        self.last_gripper_position = position
        self.last_gripper_angle = angle_degrees
        self.last_direction = direction

    def get_current_servo_position(self, channel):
        """Retrieve the current position of the servo from the controller."""
        # This will send a request to the Pololu Maestro controller to get the current position of the specified servo channel.
        cmd = bytearray([0xAA, 0x0C, 0x10, channel])
        self.serial_port.write(cmd)
        response = self.serial_port.read(2)
        if len(response) == 2:
            position = response[0] + (response[1] << 8)
            return position
        else:
            self.get_logger().warning(f'Failed to read position for channel {channel}')
            return 6000  # Fallback to a safe position if reading fails.

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
