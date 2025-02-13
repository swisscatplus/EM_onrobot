import rclpy
from rclpy.node import Node

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.get_logger().info('LocalizationNode started')

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
