import rclpy
from rclpy.node import Node
from rostak.cot_utility import CotUtility
from std_msgs.msg import String
from mavros_msgs.msg import NavSatFix

class RosCotFix(Node):
    def __init__(self):
        super().__init__("roscot_fix")
        self.declare_parameter('cot_params', '')
        self.declare_parameter('rate', 0.2)
        
        config_path = self.get_parameter('cot_params').value
        self.util = CotUtility(config_path)
        self.rate = self.get_parameter('rate').value
        self.tx = self.create_publisher(String, 'tak_tx', 1)
        self.create_subscription(NavSatFix, "mavros/global_position/global", self.publish_fix, 1)
        self.get_logger().info(str(self.util.get_config()))

    def publish_fix(self, msg):
        """Generate a status COT Event."""
        self.util.set_point(msg)
        stale_in = 2 * max(1, 1 / self.rate)
        self.msg.data = self.util.new_status_msg(stale_in)
        self.tx.publish(self.msg)

def main():
    rclpy.init()
    node = RosCotFix()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
