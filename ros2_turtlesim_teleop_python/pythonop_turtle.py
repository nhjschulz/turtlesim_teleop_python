import geometry_msgs.msg
import rclpy
from rclpy.node import Node
import geometry_msgs
import rclpy.destroyable 

class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtlesim_teleop_python')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 1.0
        msg.angular = msg.linear

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: ' + str(msg))
        self.i += 1

def main(args=None):
    print('Hi from ros2_turtlesim_teleop_python.')
    rclpy.init(args=args)
    turtle_publisher = TurtlePublisher()
    rclpy.spin(turtle_publisher)
    turtle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
