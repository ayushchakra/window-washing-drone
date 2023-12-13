import rclpy
from rclpy.node import Node
import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist, Vector3


class TeleopNode(Node):
    # This dictionary defines the linear and angular velocities that should be
    # sent to the drone based on the inputted key.
    key_to_vel = {
        "q": Twist(
            linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=1.0)
        ),
        "w": Twist(
            linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
        ),
        "e": Twist(
            linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-1.0)
        ),
        "a": Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=1.0)
        ),
        "s": Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
        ),
        "d": Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-1.0)
        ),
        "z": Twist(
            linear=Vector3(x=-0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-1.0)
        ),
        "x": Twist(
            linear=Vector3(x=-0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
        ),
        "c": Twist(
            linear=Vector3(x=-0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=1.0)
        ),
        "t": Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.5), angular=Vector3(x=0.0, y=0.0, z=0.0)
        ),
        "g": Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
        ),
        "b": Twist(
            linear=Vector3(x=0.0, y=0.0, z=-0.5), angular=Vector3(x=0.0, y=0.0, z=0.0)
        ),
    }

    def __init__(self):
        super().__init__("teloep_node")
        # Initializes the keyboard monitor in the terminal.
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        # Creates a timer that continuously checks the keyboard and sends
        # velocities to the drone
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # Creates a publisher that will send velocities to the drone.
        self.publisher = self.create_publisher(Twist, "/drone/cmd_vel", 10)

    def get_key(self):
        """
        Function to monitor the keyboard and extract any inputs from the user.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_loop(self):
        """
        Main loop that calls the keyboard monitor and send velocities based on
        any detected keyboard presses.
        """
        self.key = self.get_key()
        # If Ctrl+C is pressed, raise a KeyboardInterrupt, which stops
        # execution of the node.
        if self.key == "\x03":
            self.publisher.publish(self.key_to_vel["s"])
            raise KeyboardInterrupt
        if self.key in self.key_to_vel.keys():
            self.publisher.publish(self.key_to_vel[self.key])


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
