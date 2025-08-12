import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped 


class ForcePublish(Node):

    def __init__(self):
        super().__init__("force_publisher") 
        self.force_publisher_= self.create_publisher(WrenchStamped, "/force_torque_sensor_broadcaster/wrench",10)


    def publish_force(self):
        msg = WrenchStamped()
        msg.wrench.force.x = -1.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = 0.0

        self.force_publisher_.publish(msg)
        self.get_logger().info("force sent.") 
        print(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForcePublish()
    node.publish_force() 
    rclpy.spin(node)
    rclpy.shutdown()





