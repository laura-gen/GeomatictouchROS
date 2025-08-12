import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class PositionSubs(Node):

    def __init__(self):
        super().__init__("position_printed")
        self.pose_subscriber_= self.create_subscription(PointStamped, "/geomagic_touch/position",self.pose_callback, 10) 

    def pose_callback(self, msg: PointStamped):
        self.get_logger().info("position x:" + str(msg.point.x) + " m  " + "position y:" + str(msg.point.y) + " m  " + "position z:" + str(msg.point.z) + " m")



def main(args=None):
    rclpy.init(args=args)
    node = PositionSubs() 
    rclpy.spin(node)
    rclpy.shutdown()


