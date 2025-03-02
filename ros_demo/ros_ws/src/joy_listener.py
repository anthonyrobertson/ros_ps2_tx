import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyListener(Node):

    def __init__(self):
        super().__init__('joy_listener')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.subscription

    def joy_callback(self, msg):
        axes = ', '.join([f'{axis:.2f}' for axis in msg.axes])
        buttons = ', '.join([str(button) for button in msg.buttons])
        self.get_logger().info(f'Axes: [{axes}] Buttons: [{buttons}]')

def main(args=None):
    rclpy.init(args=args)
    joy_listener = JoyListener()
    rclpy.spin(joy_listener)
    joy_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
