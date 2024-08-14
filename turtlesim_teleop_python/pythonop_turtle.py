"""Implementation of Application main module."""

# MIT License
#
# Copyright (c) 2024 Norbert Schulz
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import math
import time

import geometry_msgs
import geometry_msgs.msg

import rclpy
import rclpy.destroyable
from rclpy.node import Node

from turtle_navigation import TNavigator

from turtlesim.srv import SetPen


class TurtleTwistPublisher(Node, TNavigator):
    """A Twist message publisher for TurtleSim."""

    def __init__(self, name="turtlesim_teleop_python"):
        """Define default constructor for class."""
        Node.__init__(self, node_name=name)
        TNavigator.__init__(self)

        self._publisher = self.create_publisher(
            geometry_msgs.msg.Twist, 'turtle1/cmd_vel', 10
        )

        self._setPen_srv = self.create_client(SetPen, '/turtle1/set_pen')

    def penup(self) -> None:
        """Lift the pen up."""
        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 3
        request.off = 1
        self._setPen_srv.call_async(request)

    pu = penup
    up = penup

    def pendown(self) -> None:
        """Put the pen down."""
        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 3
        request.off = 0
        self._setPen_srv.call_async(request)

    pd = pendown
    down = pendown

    def _rotate(self, angle):
        super()._rotate(angle)
        self._update_geometry(angle=angle)

    def _go(self, distance):
        super()._go(distance)
        self._update_geometry(distance=distance)

    def _update_geometry(self, distance: float = 0.0, angle: float = 0.0) -> None:
        """Send update messages if necessary."""
        if angle != 0.0:
            msg = geometry_msgs.msg.Twist()
            msg.angular.z = math.radians(float(angle))
            self._publisher.publish(msg)
            # self.get_logger().info("Publishing: " + str(msg))
            time.sleep(1)

        if distance != 0.0:
            msg = geometry_msgs.msg.Twist()
            msg.linear.x = float(distance)
            self._publisher.publish(msg)
            # self.get_logger().info("Publishing: " + str(msg))
            time.sleep(1)


def main(args=None):
    """Application runnable enty point."""
    rclpy.init(args=args)
    turtle = TurtleTwistPublisher()
    demo(turtle)
    turtle.destroy_node()
    rclpy.shutdown()


def demo(turtle: TurtleTwistPublisher) -> None:
    """Do some turtle fun."""
    turtle.left(90)
    turtle.forward(5)
    turtle.right(160)
    turtle.forward(5.5)
    turtle.left(160)
    turtle.forward(5)
    turtle.right(90)
    turtle.forward(3)
    turtle.backward(1.5)
    turtle.right(90)
    turtle.forward(5)
    turtle.circle(3)


if __name__ == '__main__':
    main()
