"""Implementation of turtlesim_teleop_python."""

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

from rclpy import Parameter
from rclpy.parameter_event_handler import ParameterEventHandler

from std_srvs.srv import Empty

from turtle_navigation import TNavigator, Vec2D

from turtlesim.srv import SetPen, TeleportAbsolute

PARAM_SCALE_ANGULAR = "scale_angular"  # scale angular movement
PARAM_SCALE_LINEAR = "scale_linear"  # scale forward movement
PARAM_MSG_DELAY = "delay"  # delay in milliseconds between twist commands


class TurtleTwistPublisher(Node, TNavigator):
    """A Twist message publisher for TurtleSim."""

    def __init__(self, name="turtlesim_teleop_python"):
        """Define default constructor for class."""
        Node.__init__(self, node_name=name)
        TNavigator.__init__(self)

        self.declare_parameter(PARAM_SCALE_ANGULAR, 1.0)
        self.declare_parameter(PARAM_SCALE_LINEAR, 1.0)
        self.declare_parameter(PARAM_MSG_DELAY, 1.0)

        self._publisher = self.create_publisher(
            geometry_msgs.msg.Twist, "turtle1/cmd_vel", 10
        )

        self._setPen_srv = self.create_client(SetPen, "/turtle1/set_pen")
        self._teleport_abs_srv = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        self._clear_srv = self.create_client(Empty, "/clear")

        self._scale_linear = float(
            self.get_parameter(PARAM_SCALE_LINEAR).get_parameter_value().double_value
        )
        self._scale_angular = float(
            self.get_parameter(PARAM_SCALE_ANGULAR).get_parameter_value().double_value
        )
        self._twist_delay = float(
            self.get_parameter(PARAM_MSG_DELAY).get_parameter_value().double_value
        )

    def clear(self) -> None:
        """Send clear request."""
        request = Empty.Request()
        self._clear_srv.call_async(request)

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

    def delay(self, dl=None) -> float:
        """Set or get update turtle update delay in seconds."""
        if dl is not None:
            self._twist_delay = dl

        return self._twist_delay

    def lscale(self, lsc=None) -> float:
        """Set or get scale for linear movement (1.0= meter/second)."""
        if lsc is not None:
            self._scale_linear = lsc

        return self._scale_linear

    def ascale(self, asc=None) -> float:
        """Set or get scale for angular movement (1.0= meter/second)."""
        if asc is not None:
            self._scale_angular = asc

        return self._scale_angular

    pd = pendown
    down = pendown

    def goto(self, x, y) -> None:
        super().goto(x, y)
        request = TeleportAbsolute.Request()
        request.x = float(self.xcor()) * self._scale_linear
        request.y = float(self.ycor()) * self._scale_linear
        request.theta = math.radians(self.heading() * self._scale_angular)
        self._teleport_abs_srv.call_async(request)
        rclpy.spin_once(self, timeout_sec=0.0)

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
            msg.angular.z = math.radians(float(angle) * self._scale_angular)
            self._publisher.publish(msg)

        if distance != 0.0:
            msg = geometry_msgs.msg.Twist()
            msg.linear.x = float(distance) * self._scale_linear
            self._publisher.publish(msg)

        # tiwst message delay
        wait_for_time = time.time() + self._twist_delay
        while time.time() < wait_for_time:
            rclpy.spin_once(self, timeout_sec=0.0)


def main(args=None):
    """Application runnable enty point."""
    rclpy.init(args=args)
    turtle = TurtleTwistPublisher()
    demo(turtle)
    turtle.destroy_node()
    rclpy.shutdown()


def demo(turtle: TurtleTwistPublisher) -> None:
    """Do some turtle fun."""
    turtle.penup()
    turtle.clear()
    turtle.home()
    turtle.pendown()
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
    turtle.penup()
    turtle.goto(2, 2)
    turtle.pendown()
    turtle.circle(1.5)


if __name__ == "__main__":
    main()
