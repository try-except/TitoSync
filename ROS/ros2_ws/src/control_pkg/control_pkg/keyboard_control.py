import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys
import select
import termios
import tty
import threading
from typing import Optional

class ControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        # Publisher to coms topic
        self.publisher_ = self.create_publisher(String, 'control_output', 5)
        self.get_logger().info("Control node started, listening to 'target_reference'")
        self.S1 = 0
        self.S2 = 0
        self.S3 = 0
        # Keyboard listener support (Unix terminals)
        self._old_term_settings: Optional[list] = None
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._stop_event = threading.Event()
        try:
            # Save terminal settings and enable cbreak mode so we can read single chars
            self._old_term_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self._kb_thread.start()
        except Exception as e:
            self.get_logger().warning(f'Keyboard listener not started: {e}')
        
    def publish_msg(self):
        control_msg = String()
        control_msg.data = f"DC,50,S1,{self.S1},S2,{self.S2},S3,{self.S3}\n"
        self.publisher_.publish(control_msg)
        self.get_logger().info(f'Published control_output: {control_msg.data}')

    def _keyboard_loop(self):
        """Background loop that watches stdin for single-key presses and updates S1/S2.

        Key mapping:
        - 'a' -> S1 = -5
        - 'd' -> S1 = 5
        - 'w' -> S2 = 5
        - 's' -> S2 = -5

        Pressing other keys leaves values unchanged.
        """
        self.get_logger().info('Keyboard listener thread started (keys: a,d,w,s)')
        while not self._stop_event.is_set():


            try:
                # Wait up to 0.1s for input
                dr, _, _ = select.select([sys.stdin], [], [], 0.1)
                if dr:
                    ch = sys.stdin.read(1)
                    if not ch:
                        continue
                    ch = ch.lower()
                    self.S1 = 0
                    self.S2 = 0
                    if ch == 'a':
                        self.S1 = -10
                        self.get_logger().info('Key a pressed: S1 -> -5')
                        self.publish_msg()
                    elif ch == 'd':
                        self.S1 = 10
                        self.get_logger().info('Key d pressed: S1 -> 5')
                        self.publish_msg()
                    elif ch == 'w':
                        self.S2 = 10
                        self.get_logger().info('Key w pressed: S2 -> 5')
                        self.publish_msg()
                    elif ch == 's':
                        self.S2 = -10
                        self.get_logger().info('Key s pressed: S2 -> -5')
                        self.publish_msg()
                    # Optionally handle quit
                    elif ch in ('\x03', '\x04'):
                        # Ctrl-C or Ctrl-D: request shutdown
                        self.get_logger().info('Shutdown key received, stopping...')
                        # signal rclpy shutdown by setting stop event and exit
                        try:
                            rclpy.shutdown()
                        except Exception:
                            pass
                        break
            except Exception:
                # Ignore intermittent errors (e.g., if stdin not a tty)
                break

    def destroy_node(self):
        # Signal keyboard thread to stop and restore terminal settings
        try:
            self._stop_event.set()
            if self._kb_thread.is_alive():
                self._kb_thread.join(timeout=0.5)
        except Exception:
            pass
        if self._old_term_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term_settings)
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
