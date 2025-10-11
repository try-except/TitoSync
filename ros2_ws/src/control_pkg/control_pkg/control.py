import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        # Subscriber to perception's topic
        self.subscription = self.create_subscription(
            String,
            'target_reference',
            self.listener_callback,
            10)
        self.subscription  # to prevent unused variable warning

        # Publisher to coms topic
        self.publisher_ = self.create_publisher(String, 'control_output', 10)
        self.get_logger().info("Control node started, listening to 'target_reference'")

    def listener_callback(self, msg):
        self.get_logger().info(f'Received target_reference: {msg.data}')

        # Create a control message
        control_msg = String()
        control_msg.data = f"DC,50,S1,5,S2,5,S3,10"
        self.publisher_.publish(control_msg)
        self.get_logger().info(f'Published control_output: {control_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
