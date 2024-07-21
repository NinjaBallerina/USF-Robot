import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('Joystick Control Node Initialized')

    def joy_callback(self, msg):
        self.get_logger().info(f'Joystick input: {msg.axes} {msg.buttons}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
