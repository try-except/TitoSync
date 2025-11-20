import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time, serial

class ComsNode(Node):
    def __init__(self):
        super().__init__('coms')
        self.subscription = self.create_subscription(
            String,
            'control_output',
            self.listener_callback,
            10)
        self.subscription
        self.get_logger().info("Coms node started, listening to 'control_output'")
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received from control: {msg.data}')
        self.ser.write(msg.data.encode('ascii'))

def main(args=None):
    rclpy.init(args=args)
    node = ComsNode()
    rclpy.spin(node)
    node.destroy_node()
    node.ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
