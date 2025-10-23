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

        self.x = 0
        self.y = 0
        self.xt = 0
        self.yt = 0


    def listener_callback(self, msg):
        target_tuple = ast.literal_eval(msg.data)  # '(x, y, x1, y1)'
        self.x, self.y, self.xt, self.yt = target_tuple
        self.get_logger().info(f'Recibido: x={self.x}, y={self.y}, xt={self.x1}, yt={self.y1}')

        # Create a control message
        control_msg = String()
        control_msg.data = f"DC,50,S1,5,S2,5,S3,10"
        self.publisher_.publish(control_msg)
        self.get_logger().info(f'Published control_output: {control_msg.data}')

    def control M_ang (self):
        pass
    def control M_avance (self):
        pass
    def control M_bajada(self):
        pass
    def control M_taladro(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
