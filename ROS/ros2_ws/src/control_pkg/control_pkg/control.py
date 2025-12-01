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
        # read msg and separate values (refx, refy, self.x, self.y, dist, angulo_deg), its a string
        target_str = msg.data
        target_tuple = eval(target_str)





        self.x, self.y, self.xt, self.yt, dist, angle = target_tuple
        self.get_logger().info(f'Recibido: x={self.x}, y={self.y}, xt={self.xt}, yt={self.yt}')
        if dist > 360:
            control_msg = String()
            if angle > 5:
                control_msg.data =  f"DC,50,S1,0,S2,-{int(int(angle)/1)},S3,-10"
            elif angle < -5:
                control_msg.data =  f"DC,50,S1,0,S2,{int(int(abs(angle))/1)},S3,10"
        # Create a control message
        if  dist > 200 and abs(angle) >= 5:
            control_msg = String()
            if angle > 5:
                control_msg.data =  f"DC,50,S1,0,S2,-{int(int(angle)/1)},S3,-10"
            elif angle < -5:
                control_msg.data =  f"DC,50,S1,0,S2,{int(int(abs(angle))/1)},S3,10"

        elif dist < 80 or (abs(angle) > 70 and dist < 110):
            control_msg = String()
            control_msg.data = "DC,0,S1,0,S2,0,S3,0"
        else:
            control_msg = String()
            if  abs(angle) > 90:
                #control_msg.data = f"DC,50,S1,{int(int(dist)/6)},S2,5,S3,-10"
                pass
            else:
                #control_msg.data = f"DC,50,S1,-{int(int(dist)/6)},S2,5,S3,10"
                pass
            control_msg.data = "DC,0,S1,0,S2,0,S3,0" #borrar
        self.publisher_.publish(control_msg)
        self.get_logger().info(f'Published control_output: {control_msg.data}')

    def control_M_ang (self):
        pass
    def control_M_avance (self):
        pass
    def control_M_bajada(self):
        pass
    def control_M_taladro(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
