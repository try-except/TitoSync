import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # simple type for now

class Camara(Node):
    def __init__(self):
        super().__init__('camara')
        self.publisher_ = self.create_publisher(String, 'target_reference', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # publish every second
        self.get_logger().info("Camara node started, publishing to 'target_reference'")

    def timer_callback(self):
        # Example tuple: (x, y)
        target = (42, 17)
        msg = String()
        msg.data = str(target)  # publish as string for now
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Camara()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
