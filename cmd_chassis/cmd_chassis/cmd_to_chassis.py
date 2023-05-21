import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rm_interfaces.msg import ChassisCmd


class CmdChassisNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_chassis")
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.msg_callback, 10)
        self.chassis_pub = self.create_publisher(ChassisCmd, "chassis_cmd", 10)
        self.chassis_msg = ChassisCmd()
        self.chassis_msg.type = 1

    def msg_callback(self, msg):
        self.chassis_msg.twist = msg
        self.chassis_pub.publish(self.chassis_msg)


def main(args=None):
    rclpy.init(args=args)

    node = CmdChassisNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
