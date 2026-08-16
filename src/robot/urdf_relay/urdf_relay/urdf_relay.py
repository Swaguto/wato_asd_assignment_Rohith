
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

REPUBLISH_PERIOD = 1.0

class UrdfRelay(Node):

    def __init__(self):
        super().__init__("urdf_relay")
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, "/robot_description_live", 10)
        self.last = None
        self.create_subscription(String, "/robot_description", self.cb, latch_qos)
        self.create_timer(REPUBLISH_PERIOD, self.tick)

    def cb(self, msg):
        self.last = msg
        self.pub.publish(msg)

    def tick(self):
        if self.last is not None:
            self.pub.publish(self.last)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(UrdfRelay())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
