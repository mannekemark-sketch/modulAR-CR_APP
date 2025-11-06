import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Arduino serial port of Mirte (in our case the USB)
        self.ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 9600, timeout=1)
        # Listen to what is published on door_cmd
        self.subscription = self.create_subscription(
            String,
            'door_cmd',
            self.listener_callback,
            10
        )
    #ask for the user to publish a letter that is either o or c
    def listener_callback(self, msg):
        command = msg.data.strip().lower()
        if command in ['o', 'c']:
            self.ser.write(command.encode())
            self.get_logger().info(f"Sent '{command}' to Arduino")
        else:
            self.get_logger().warn(f"Ignored invalid command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()