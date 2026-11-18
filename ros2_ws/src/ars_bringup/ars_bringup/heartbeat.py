"""Heartbeat node: publishes a counter at 1 Hz on /heartbeat.

Purpose: a smoke test that ROS 2 builds, runs and discovers peers inside the container.
It controls nothing and carries no safety function.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Heartbeat(Node):
    def __init__(self) -> None:
        super().__init__('heartbeat')
        self._publisher = self.create_publisher(String, 'heartbeat', 10)
        self._count = 0
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        self._count += 1
        msg = String()
        msg.data = f'alive {self._count}'
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Heartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On Ctrl-C, rclpy may already have shut the context down (it installs its own
        # SIGINT handler). Calling shutdown twice raises RCLError, so guard it.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
