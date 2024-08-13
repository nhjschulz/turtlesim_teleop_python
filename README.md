# turtlesim_teleop_python
ROS2 TurtleSim  Python Controller



# Interactive Test
```python
import sys
sys.path.insert(0, ".")

import rclpy
from pythonop_turtle import TurtleTwistPublisher

rclpy.init() 
turtle = TurtleTwistPublisher()

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

turtle.destroy_node()
rclpy.shutdown()
```