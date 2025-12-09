---
sidebar_position: 2
---

# Python Agents with rclpy

## Introduction

In the previous chapter, we explored the fundamental concepts of ROS 2 middleware, including nodes, topics, services, and actions. Now, we'll dive into how to implement these concepts using Python through the rclpy library, which provides Python bindings for ROS 2. This is particularly important for creating AI agents that can control robots.

## Understanding rclpy

**rclpy** is the Python client library for ROS 2. It allows Python programs to interact with ROS 2 systems, enabling you to create nodes, publish and subscribe to topics, and provide or use services. This library is crucial for developing AI agents that can interface with robotic systems.

### Installing and Setting Up rclpy

To use rclpy, you need to have ROS 2 installed on your system. The most common distribution for development is ROS 2 Humble Hawksbill. Once ROS 2 is installed, rclpy comes as part of the core installation.

```bash
# Make sure ROS 2 is sourced in your terminal
source /opt/ros/humble/setup.bash
```

## Creating a Simple Publisher Node

Let's start by creating a simple publisher node that publishes messages to a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Simple Subscriber Node

Now, let's create a subscriber node that receives messages from the publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## How rclpy Connects AI Agents to Robot Controllers

The power of rclpy lies in its ability to connect AI agents with robot controllers. Here's how this works:

1. **Sensor Data Interface**: AI agents can subscribe to sensor topics to receive data from robot sensors (cameras, lidar, IMU, etc.)
2. **Control Command Interface**: AI agents can publish control commands to robot actuators
3. **Planning Interface**: AI agents can use services to request path planning or other computational tasks

### Example: Simple AI Agent Controller

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class AIController(Node):
    def __init__(self):
        super().__init__('ai_controller')

        # Publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for laser scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Store sensor data
        self.scan_data = None

    def scan_callback(self, msg):
        self.scan_data = msg.ranges

    def control_loop(self):
        if self.scan_data is None:
            return

        # Simple AI behavior: avoid obstacles
        cmd_vel = Twist()

        # Check for obstacles in front
        front_distances = self.scan_data[:30] + self.scan_data[-30:]
        min_distance = min(front_distances) if front_distances else float('inf')

        if min_distance > 1.0:  # No obstacle nearby
            cmd_vel.linear.x = 0.5  # Move forward
            cmd_vel.angular.z = 0.0
        else:  # Obstacle detected
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn right

        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    ai_controller = AIController()
    rclpy.spin(ai_controller)
    ai_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simple Control Loop Example

A control loop is fundamental in robotics for continuously processing sensor data and sending control commands. Here's a more complete example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time

class ControlLoopNode(Node):
    def __init__(self):
        super().__init__('control_loop_node')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sensor_sub = self.create_subscription(
            Float64, 'sensor_input', self.sensor_callback, 10
        )

        # Control parameters
        self.target_value = 0.0
        self.current_value = 0.0
        self.control_active = True

        # Timer for control loop (10 Hz)
        self.control_timer = self.create_timer(0.1, self.run_control_loop)

    def sensor_callback(self, msg):
        self.current_value = msg.data

    def run_control_loop(self):
        if not self.control_active:
            return

        # Simple proportional controller
        error = self.target_value - self.current_value
        control_output = 0.1 * error  # Simple P controller

        # Create and publish control command
        cmd = Twist()
        cmd.linear.x = max(min(control_output, 1.0), -1.0)  # Limit output
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlLoopNode()

    # Set a target value
    control_node.target_value = 5.0

    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info('Stopping control loop...')
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for rclpy Development

1. **Use proper shutdown procedures** to clean up resources
2. **Handle exceptions** gracefully to prevent node crashes
3. **Use appropriate QoS settings** for different types of data
4. **Implement proper logging** for debugging and monitoring
5. **Structure your code** using classes that inherit from Node

## Summary

In this chapter, we've learned how to use rclpy to create Python-based ROS 2 nodes that can serve as AI agents controlling robots. We've covered:

- Basic publisher and subscriber patterns
- How AI agents interface with robot controllers
- A complete example of an AI-based obstacle avoidance controller
- Best practices for control loop implementation

In the next chapter, we'll explore URDF (Unified Robot Description Format) and how to create humanoid robot models for use in ROS 2.

## References

1. ROS 2 Documentation. (2023). rclpy: Python Client Library for ROS 2. https://docs.ros.org/en/humble/p/rclpy/
2. Cousins, W. (2022). Programming Robots with ROS: A Practical Introduction to the Robot Operating System. O'Reilly Media.