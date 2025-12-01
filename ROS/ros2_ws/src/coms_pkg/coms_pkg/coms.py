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

        self.get_logger().info("Coms node started")

        # Abrir serial con delay
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)

        # rate limit envío a 20Hz
        self.last_send_time = 0


    def listener_callback(self, msg):

        # Rate limit
        now = time.time()
        if now - self.last_send_time < 0.05:
            return
        self.last_send_time = now

        texto = msg.data + "\n"

        try:
            self.ser.write(texto.encode('utf-8'))
            respuesta = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except serial.SerialException:
            self.get_logger().error("Serial disconnected, reopening...")
            try:
                self.ser.close()
            except:
                pass
            time.sleep(1)
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    node = ComsNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    node.ser.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()