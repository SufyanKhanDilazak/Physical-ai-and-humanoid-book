---
title: "Navigation with Isaac ROS and Nav2"
sidebar_label: "Navigation with Isaac ROS and Nav2"
description: "Learning to set up navigation with Isaac ROS and Nav2 for humanoid robot path planning and obstacle avoidance."
---

# Navigation with Isaac ROS and Nav2

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up and configure Nav2 for humanoid robot navigation
- Integrate Isaac ROS perception with Nav2 navigation stack
- Implement VSLAM (Visual Simultaneous Localization and Mapping) for humanoid robots
- Configure navigation parameters for bipedal locomotion
- Plan and execute safe paths in dynamic environments
- Handle navigation recovery behaviors for humanoid robots

## Introduction

Navigation is a critical capability for humanoid robots, enabling them to move autonomously through complex environments. The combination of Isaac ROS perception capabilities with the Nav2 navigation stack provides a powerful framework for humanoid robot navigation. This chapter covers the integration of visual perception with navigation planning to create robust autonomous navigation systems.

Unlike wheeled robots, humanoid robots present unique challenges for navigation due to their bipedal locomotion, balance requirements, and complex kinematics. The navigation system must account for the robot's dynamic stability, footstep planning, and the need for precise positioning for manipulation tasks.

## Understanding Nav2 Architecture

### Nav2 System Overview

Nav2 (Navigation Stack 2) is the next-generation navigation framework for ROS 2, designed specifically for autonomous mobile robots. The architecture consists of several key components:

```
[Navigation Action Server] → [Planner Server] → [Controller Server] → [Recovery Server]
        ↓                         ↓                    ↓                  ↓
[Behavior Trees] ←→ [Lifecycle Manager] ←→ [Transforms] ←→ [Sensors]
```

### Key Nav2 Components

1. **Planner Server**: Global and local path planners
2. **Controller Server**: Local trajectory controller
3. **Recovery Server**: Behavior for getting unstuck
4. **Lifecycle Manager**: Component state management
5. **Behavior Trees**: Task execution and decision making

### Nav2 vs Traditional Navigation

| Feature | Traditional Nav | Nav2 |
|---------|----------------|------|
| Architecture | Static | Lifecycle-based |
| Configuration | Launch files | YAML parameters |
| Flexibility | Limited | Highly configurable |
| Recovery | Basic | Advanced behavior trees |
| Plugins | C++ only | C++/Python plugins |

## Setting Up Nav2 for Humanoid Robots

### Installation and Dependencies

```bash
# Install Nav2 packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-controller ros-humble-nav2-planners
sudo apt install ros-humble-nav2-behaviors ros-humble-nav2-lifecycle-manager

# Install Isaac ROS navigation packages (if available)
sudo apt install ros-humble-isaac-ros-nav2
```

### Basic Nav2 Configuration

```yaml
# config/nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.5
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
```

### Humanoid-Specific Navigation Parameters

```yaml
# config/humanoid_nav2_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_footprint  # Important for bipedal robots
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05  # Higher resolution for precise footstep planning
      robot_radius: 0.4  # Account for robot's balance envelope
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        enabled: True
        voxel_size: 0.05
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.4
      resolution: 0.1  # Lower resolution for global planning
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        map_subscribe_transient_local: True
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 24
      control_freq: 20.0
      horizon: 1.2
      dt: 0.05
      omni_robot: false  # Not an omni-directional robot
      ackermann_robot: false
      rcl_topic_qos_profile: "system_default"
      xy_goal_tolerance: 0.25  # Tighter tolerance for humanoid manipulation
      yaw_goal_tolerance: 0.1  # Precise orientation for manipulation
      stateful: true
      reset_period: 0.0
      velocity_scaling_tolerance: 0.1
      max_robot_pose_search_dist: 1.0
      max_robot_pose_search_angle: 1.57
      transform_tolerance: 0.1
      use_interpolation: true
      publish_cost_grid_pc: false
      # Humanoid-specific parameters
      max_speed: 0.5  # Conservative speed for balance
      min_speed: 0.1
      max_accel: 0.2
      max_decel: 0.3
```

## Isaac ROS Integration for Navigation

### Isaac ROS Perception Stack

Isaac ROS provides advanced perception capabilities that can enhance navigation:

```python
# launch/isaac_ros_nav2.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Navigation parameters
    nav2_params = os.path.join(
        get_package_share_directory('my_robot_navigation'),
        'config',
        'humanoid_nav2_params.yaml'
    )

    # Isaac ROS perception nodes
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Visual SLAM node
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'enable_rectified_pose': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_observations_view': True,
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                }],
                remappings=[
                    ('/visual_slam/camera/left/image_rect', '/camera/left/image_rect_color'),
                    ('/visual_slam/camera/right/image_rect', '/camera/right/image_rect_color'),
                    ('/visual_slam/camera/left/camera_info', '/camera/left/camera_info'),
                    ('/visual_slam/camera/right/camera_info', '/camera/right/camera_info'),
                ],
            ),

            # Stereo dense reconstruction
            ComposableNode(
                package='isaac_ros_stereo_dnn',
                plugin='nvidia::isaac_ros::stereo_dnn::StereoDnnNode',
                name='stereo_dnn',
                parameters=[{
                    'network_info': {
                        'input_tensor_names': ['input'],
                        'input_binding_names': ['input'],
                        'output_tensor_names': ['output'],
                        'output_binding_names': ['output'],
                    },
                    'dnn_processing_size': [512, 256],
                    'enable_padding': True,
                    'max_workspace_size': 1073741824,  # 1GB
                }],
                remappings=[
                    ('left_image', '/camera/left/image_rect_color'),
                    ('right_image', '/camera/right/image_rect_color'),
                ],
            ),
        ],
        output='screen',
    )

    # Nav2 nodes
    nav2_nodes = [
        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                       {'autostart': True},
                       {'node_names': ['map_server',
                                     'planner_server',
                                     'controller_server',
                                     'recoveries_server',
                                     'bt_navigator',
                                     'waypoint_follower']}]
        ),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': True}]
        ),

        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': True}]
        ),

        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': True}]
        ),

        # Behavior tree navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': True}]
        ),
    ]

    return LaunchDescription([
        perception_container
    ] + nav2_nodes)
```

### VSLAM Integration with Nav2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np

class VSLAMNav2Integrator(Node):
    def __init__(self):
        super().__init__('vslam_nav2_integrator')

        # Subscriptions
        self.vslam_pose_sub = self.create_subscription(
            PoseStamped, '/visual_slam/pose', self.vslam_pose_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.amcl_pose_pub = self.create_publisher(
            PoseStamped, '/amcl_pose', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/integrated_odom', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.vslam_pose = None
        self.odom_pose = None
        self.initialized = False

        # Timer for pose integration
        self.timer = self.create_timer(0.1, self.integrate_poses)

    def vslam_pose_callback(self, msg):
        """Handle VSLAM pose updates"""
        self.vslam_pose = msg
        if not self.initialized:
            self.initialize_system(msg)

    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.odom_pose = msg

    def initialize_system(self, initial_pose):
        """Initialize the integrated navigation system"""
        self.map_to_odom_transform = initial_pose.pose
        self.initialized = True
        self.get_logger().info('VSLAM-Nav2 integration initialized')

    def integrate_poses(self):
        """Integrate VSLAM and odometry poses"""
        if not self.initialized or not self.vslam_pose or not self.odom_pose:
            return

        # Fuse VSLAM and odometry using a simple complementary filter
        alpha = 0.1  # VSLAM trust factor (lower = trust odometry more)

        # Get current poses
        vslam_pos = np.array([
            self.vslam_pose.pose.position.x,
            self.vslam_pose.pose.position.y,
            self.vslam_pose.pose.position.z
        ])

        odom_pos = np.array([
            self.odom_pose.pose.pose.position.x,
            self.odom_pose.pose.pose.position.y,
            self.odom_pose.pose.pose.position.z
        ])

        # Fuse positions
        fused_pos = alpha * vslam_pos + (1 - alpha) * odom_pos

        # Create integrated pose
        integrated_pose = PoseStamped()
        integrated_pose.header.stamp = self.get_clock().now().to_msg()
        integrated_pose.header.frame_id = 'map'
        integrated_pose.pose.position.x = float(fused_pos[0])
        integrated_pose.pose.position.y = float(fused_pos[1])
        integrated_pose.pose.position.z = float(fused_pos[2])

        # Use VSLAM orientation (more reliable for rotation)
        integrated_pose.pose.orientation = self.vslam_pose.pose.orientation

        # Publish integrated pose
        self.amcl_pose_pub.publish(integrated_pose)

        # Broadcast transform
        self.broadcast_transform(integrated_pose)

    def broadcast_transform(self, pose_stamped):
        """Broadcast the integrated transform"""
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = pose_stamped.pose.position.x
        t.transform.translation.y = pose_stamped.pose.position.y
        t.transform.translation.z = pose_stamped.pose.position.z
        t.transform.rotation = pose_stamped.pose.orientation

        self.tf_broadcaster.sendTransform(t)
```

## Footstep Planning for Bipedal Navigation

### Humanoid Navigation Considerations

Humanoid robots require special consideration for navigation due to their bipedal nature:

```python
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class FootstepPlanner:
    def __init__(self):
        self.foot_separation = 0.3  # Distance between feet
        self.step_length = 0.4      # Maximum step length
        self.step_height = 0.05     # Step height for clearance
        self.support_polygon = self.calculate_support_polygon()

    def calculate_support_polygon(self):
        """Calculate the support polygon for balance"""
        # For a humanoid, the support polygon is between the feet
        return [
            Point(x=-0.1, y=-self.foot_separation/2, z=0.0),
            Point(x=0.1, y=-self.foot_separation/2, z=0.0),
            Point(x=0.1, y=self.foot_separation/2, z=0.0),
            Point(x=-0.1, y=self.foot_separation/2, z=0.0)
        ]

    def plan_footsteps(self, path, robot_pose):
        """Plan footsteps along a navigation path"""
        footsteps = []

        # Convert path to footsteps
        for i in range(1, len(path)):
            current_step = path[i-1]
            next_step = path[i]

            # Calculate required step
            step_vector = np.array([next_step.x - current_step.x,
                                  next_step.y - current_step.y])
            step_distance = np.linalg.norm(step_vector)

            if step_distance > self.step_length:
                # Need to break into multiple steps
                num_steps = int(np.ceil(step_distance / self.step_length))
                for j in range(num_steps):
                    intermediate_pos = current_step + (step_vector / num_steps) * j
                    footsteps.append(self.create_footstep(intermediate_pos, j % 2))
            else:
                footsteps.append(self.create_footstep(next_step, i % 2))

        return footsteps

    def create_footstep(self, position, foot_id):
        """Create a footstep at the given position"""
        footstep = {
            'position': position,
            'foot_id': foot_id,  # 0 for left, 1 for right
            'timestamp': None,
            'z_offset': self.step_height
        }
        return footstep

    def validate_footstep(self, footstep, costmap):
        """Validate if a footstep is safe to execute"""
        # Check if footstep is in collision
        grid_x = int((footstep['position'].x - costmap.info.origin.position.x) / costmap.info.resolution)
        grid_y = int((footstep['position'].y - costmap.info.origin.position.y) / costmap.info.resolution)

        if (0 <= grid_x < costmap.info.width and 0 <= grid_y < costmap.info.height):
            index = grid_y * costmap.info.width + grid_x
            cost = costmap.data[index]

            # Check if cost is acceptable (not in obstacle)
            return cost < 253  # 254 and 255 are lethal obstacles

        return False

    def visualize_footsteps(self, footsteps):
        """Create visualization markers for footsteps"""
        marker_array = MarkerArray()

        for i, footstep in enumerate(footsteps):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "footsteps"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = footstep['position']
            marker.pose.position.z = footstep['z_offset']
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.05

            marker.color.r = 1.0 if footstep['foot_id'] == 0 else 0.0  # Red for left, Blue for right
            marker.color.g = 0.0
            marker.color.b = 0.0 if footstep['foot_id'] == 0 else 1.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        return marker_array
```

### Navigation with Balance Constraints

```python
class BalanceAwareNavigator:
    def __init__(self):
        self.balance_margin = 0.1  # Safety margin for balance
        self.max_lean_angle = 15.0  # Maximum lean angle in degrees
        self.center_of_mass_height = 0.8  # Height of CoM in meters

    def check_balance_feasibility(self, path, robot_state):
        """Check if the path is feasible given balance constraints"""
        for point in path:
            if not self.is_balance_safe(point, robot_state):
                return False
        return True

    def is_balance_safe(self, target_pose, robot_state):
        """Check if target pose maintains robot balance"""
        # Calculate center of mass projection
        com_proj = self.calculate_com_projection(robot_state)

        # Calculate support polygon for current stance
        support_poly = self.calculate_support_polygon(robot_state)

        # Check if CoM projection is within support polygon
        return self.point_in_polygon(com_proj, support_poly)

    def calculate_com_projection(self, robot_state):
        """Calculate center of mass projection on ground plane"""
        # Simplified CoM calculation
        return np.array([robot_state.position.x, robot_state.position.y])

    def calculate_support_polygon(self, robot_state):
        """Calculate support polygon based on foot positions"""
        # This would consider actual foot positions
        left_foot = np.array([robot_state.position.x - 0.15, robot_state.position.y + 0.15])
        right_foot = np.array([robot_state.position.x + 0.15, robot_state.position.y - 0.15])

        # Create convex hull of support polygon
        return [left_foot, right_foot]

    def point_in_polygon(self, point, polygon):
        """Check if point is inside polygon using ray casting"""
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def modify_path_for_balance(self, original_path, robot_state):
        """Modify path to ensure balance constraints are met"""
        modified_path = []
        current_state = robot_state

        for target_point in original_path:
            # Find closest balance-safe point
            safe_point = self.find_balance_safe_point(target_point, current_state)
            if safe_point:
                modified_path.append(safe_point)
                current_state = self.update_robot_state(current_state, safe_point)
            else:
                # If no safe point found, return None to indicate path is not feasible
                return None

        return modified_path

    def find_balance_safe_point(self, target_point, robot_state):
        """Find the closest point to target that maintains balance"""
        # Search in a small area around the target point
        search_radius = 0.2
        resolution = 0.05

        best_point = None
        min_distance = float('inf')

        for dx in np.arange(-search_radius, search_radius, resolution):
            for dy in np.arange(-search_radius, search_radius, resolution):
                candidate_point = np.array([
                    target_point[0] + dx,
                    target_point[1] + dy
                ])

                if self.is_balance_safe(candidate_point, robot_state):
                    distance = np.linalg.norm(candidate_point - target_point)
                    if distance < min_distance:
                        min_distance = distance
                        best_point = candidate_point

        return best_point
```

## Behavior Trees for Humanoid Navigation

### Custom Behavior Tree Nodes

```python
import py_trees
from py_trees.behaviours import SuccessAlways
from py_trees_ros.actions import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class CheckBalance(py_trees.behaviour.Behaviour):
    """Check if robot is in safe balance state"""

    def __init__(self, name="CheckBalance"):
        super(CheckBalance, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name="CheckBalance")
        self.blackboard.register_key("robot_balance_state", access=py_trees.common.Access.READ)

    def update(self):
        # Check balance state from blackboard
        balance_state = self.blackboard.robot_balance_state

        if balance_state == "STABLE":
            return py_trees.common.Status.SUCCESS
        elif balance_state == "UNSTABLE":
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

class ExecuteFootstepPlan(py_trees.behaviour.Behaviour):
    """Execute a planned sequence of footsteps"""

    def __init__(self, name="ExecuteFootstepPlan"):
        super(ExecuteFootstepPlan, self).__init__(name)
        self.footstep_executor = None  # Initialize with actual footstep executor

    def update(self):
        # Execute next footstep in plan
        if self.footstep_executor.has_more_steps():
            success = self.footstep_executor.execute_next_step()
            if success:
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

class HumanoidNavigateTree:
    """Behavior tree for humanoid navigation with balance awareness"""

    def __init__(self):
        self.root = self.create_tree()

    def create_tree(self):
        # Main sequence
        root = py_trees.composites.Sequence(name="HumanoidNavigation")

        # Check if robot is stable before navigation
        check_balance = CheckBalance()

        # Plan path using Nav2
        plan_path = SuccessAlways(name="PlanPath")

        # Plan footsteps for bipedal locomotion
        plan_footsteps = SuccessAlways(name="PlanFootsteps")

        # Execute footsteps
        execute_footsteps = ExecuteFootstepPlan()

        # Navigate to goal
        navigate = ActionClient(
            name="NavigateToPose",
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
            action_goal=PoseStamped()
        )

        # Add to tree
        root.add_children([check_balance, plan_path, plan_footsteps, execute_footsteps, navigate])

        return root

    def tick(self):
        """Tick the behavior tree"""
        self.root.tick_once()
        return self.root.status
```

## Recovery Behaviors for Humanoid Robots

### Specialized Recovery Behaviors

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
import math

class HumanoidRecoveryBehaviors(Node):
    def __init__(self):
        super().__init__('humanoid_recovery_behaviors')

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Recovery parameters
        self.max_recovery_attempts = 3
        self.recovery_timeout = 30.0  # seconds

    def balance_recovery(self):
        """Recovery behavior for balance issues"""
        self.get_logger().info('Executing balance recovery')

        # Stop all movement
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Wait for balance to stabilize
        balance_stable = self.wait_for_balance_stable(5.0)

        if balance_stable:
            self.get_logger().info('Balance recovered')
            return True
        else:
            self.get_logger().error('Balance recovery failed')
            return False

    def step_back_recovery(self):
        """Recovery by stepping back"""
        self.get_logger().info('Executing step back recovery')

        # Execute backward step
        cmd = Twist()
        cmd.linear.x = -0.2  # Move backward slowly
        cmd.angular.z = 0.0

        start_time = self.get_clock().now()
        timeout = rclpy.time.Duration(seconds=2.0)

        while (self.get_clock().now() - start_time) < timeout:
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        return True

    def alternative_path_recovery(self, current_pose, goal_pose):
        """Recovery by finding alternative path"""
        self.get_logger().info('Executing alternative path recovery')

        # Calculate alternative goal slightly offset from original
        alternative_goal = self.calculate_alternative_goal(current_pose, goal_pose)

        if alternative_goal:
            return self.navigate_to_pose(alternative_goal)
        else:
            return False

    def calculate_alternative_goal(self, current_pose, original_goal):
        """Calculate alternative goal position"""
        # Calculate vector from current to original goal
        dx = original_goal.pose.position.x - current_pose.pose.position.x
        dy = original_goal.pose.position.y - current_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.1:  # Already very close
            return None

        # Calculate perpendicular offset
        offset_distance = 0.3  # 30cm offset
        perpendicular_dx = -dy / distance * offset_distance
        perpendicular_dy = dx / distance * offset_distance

        # Create alternative goal
        alternative_goal = PoseStamped()
        alternative_goal.header.frame_id = original_goal.header.frame_id
        alternative_goal.header.stamp = self.get_clock().now().to_msg()
        alternative_goal.pose.position.x = original_goal.pose.position.x + perpendicular_dx
        alternative_goal.pose.position.y = original_goal.pose.position.y + perpendicular_dy
        alternative_goal.pose.position.z = original_goal.pose.position.z

        # Keep same orientation
        alternative_goal.pose.orientation = original_goal.pose.orientation

        return alternative_goal

    def navigate_to_pose(self, pose_stamped):
        """Navigate to a specific pose"""
        self.nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        future = self.nav_client.send_goal_async(goal_msg)

        # Wait for result with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.recovery_timeout)

        if future.result() is not None:
            result = future.result().result
            return result is not None
        else:
            return False

    def wait_for_balance_stable(self, timeout):
        """Wait for robot balance to become stable"""
        start_time = self.get_clock().now()
        timeout_duration = rclpy.time.Duration(seconds=timeout)

        while (self.get_clock().now() - start_time) < timeout_duration:
            # Check balance state (this would connect to actual balance sensor)
            balance_stable = self.check_balance_state()

            if balance_stable:
                return True

            rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def check_balance_state(self):
        """Check current balance state of robot"""
        # This would interface with actual balance sensors/controllers
        # For simulation, return True
        return True
```

## Performance Optimization and Tuning

### Navigation Parameter Tuning

```python
class NavigationParameterTuner:
    def __init__(self):
        self.default_params = {
            'planner_frequency': 5.0,
            'controller_frequency': 20.0,
            'max_vel_x': 0.5,
            'min_vel_x': 0.1,
            'max_vel_theta': 0.5,
            'min_vel_theta': 0.1,
            'acc_lim_x': 0.5,
            'acc_lim_theta': 0.5,
            'xy_goal_tolerance': 0.25,
            'yaw_goal_tolerance': 0.1,
            'holonomic_robot': False,
            'oscillation_distance': 0.05
        }

        self.tuning_ranges = {
            'max_vel_x': (0.1, 1.0),
            'min_vel_x': (0.05, 0.3),
            'acc_lim_x': (0.1, 1.0),
            'xy_goal_tolerance': (0.1, 0.5),
            'yaw_goal_tolerance': (0.05, 0.3)
        }

    def tune_for_environment(self, environment_type):
        """Tune navigation parameters for specific environment"""
        params = self.default_params.copy()

        if environment_type == 'cluttered':
            # Reduce speeds for tight spaces
            params['max_vel_x'] = 0.3
            params['acc_lim_x'] = 0.2
            params['xy_goal_tolerance'] = 0.15
        elif environment_type == 'open':
            # Increase speeds for open spaces
            params['max_vel_x'] = 0.7
            params['acc_lim_x'] = 0.8
        elif environment_type == 'dynamic':
            # Adjust for moving obstacles
            params['max_vel_x'] = 0.4
            params['oscillation_distance'] = 0.1
        elif environment_type == 'precision':
            # For precise manipulation positioning
            params['max_vel_x'] = 0.2
            params['xy_goal_tolerance'] = 0.1
            params['yaw_goal_tolerance'] = 0.05

        return params

    def adaptive_parameter_control(self, robot_state, environment_state):
        """Adjust parameters based on current conditions"""
        params = self.default_params.copy()

        # Adjust based on balance state
        if robot_state.balance_confidence < 0.7:
            params['max_vel_x'] *= 0.5  # Reduce speed when balance is uncertain
            params['acc_lim_x'] *= 0.7

        # Adjust based on obstacle proximity
        if environment_state.closest_obstacle_distance < 0.5:
            params['max_vel_x'] *= 0.7  # Slow down near obstacles
            params['xy_goal_tolerance'] = 0.15  # Tighter tolerance near obstacles

        # Adjust based on task requirements
        if environment_state.near_manipulation_area:
            params['max_vel_x'] = 0.2  # Very slow for precision tasks
            params['xy_goal_tolerance'] = 0.1
            params['yaw_goal_tolerance'] = 0.05

        return params

    def evaluate_navigation_performance(self, navigation_log):
        """Evaluate navigation performance and suggest improvements"""
        metrics = {
            'success_rate': self.calculate_success_rate(navigation_log),
            'average_time': self.calculate_average_time(navigation_log),
            'path_efficiency': self.calculate_path_efficiency(navigation_log),
            'oscillation_count': self.count_oscillations(navigation_log),
            'collision_count': self.count_collisions(navigation_log)
        }

        recommendations = self.generate_recommendations(metrics)
        return metrics, recommendations

    def generate_recommendations(self, metrics):
        """Generate parameter recommendations based on performance"""
        recommendations = []

        if metrics['success_rate'] < 0.8:
            recommendations.append("Increase xy_goal_tolerance or adjust controller parameters")

        if metrics['oscillation_count'] > 5:
            recommendations.append("Increase oscillation_distance or adjust controller gains")

        if metrics['average_time'] > 60:  # seconds
            recommendations.append("Increase planner/controller frequencies or max velocities")

        if metrics['path_efficiency'] < 0.7:  # ratio of direct to actual path
            recommendations.append("Tune global planner parameters for better path planning")

        return recommendations
```

## Troubleshooting and Common Issues

### 1. Localization Problems
- **Issue**: Robot loses track of position in large/featureless environments
- **Solution**: Improve map quality, add visual markers, tune AMCL parameters

### 2. Navigation Oscillation
- **Issue**: Robot oscillates back and forth in narrow passages
- **Solution**: Increase oscillation distance, adjust controller parameters

### 3. Balance-Related Failures
- **Issue**: Navigation fails due to balance constraints
- **Solution**: Implement balance-aware path planning, reduce speeds

### 4. Footstep Planning Issues
- **Issue**: Planned footsteps are not executable
- **Solution**: Validate footsteps against robot kinematics, adjust step parameters

## Summary

Navigation with Isaac ROS and Nav2 provides powerful capabilities for humanoid robot autonomy. In this chapter, we covered:

- Nav2 architecture and configuration for humanoid robots
- Isaac ROS integration for enhanced perception and navigation
- Footstep planning and balance-aware navigation
- Behavior trees for complex navigation tasks
- Recovery behaviors specific to humanoid robots
- Performance optimization and parameter tuning
- Troubleshooting common navigation issues

The integration of visual perception with navigation planning creates robust autonomous systems capable of navigating complex environments while maintaining the balance and precision required for humanoid robotics.

## Learning Outcomes

After completing this chapter, you should be able to:
- Configure and tune Nav2 for humanoid robot navigation
- Integrate Isaac ROS perception with Nav2 navigation stack
- Implement footstep planning for bipedal locomotion
- Design behavior trees for complex navigation tasks
- Implement recovery behaviors for humanoid-specific failures
- Optimize navigation parameters for different environments and tasks

## References

1. ROS Navigation Working Group. (2024). *Nav2 User Guide and Best Practices*. Open Robotics.
2. Smith, J., et al. (2023). "Bipedal Navigation: Challenges and Solutions." *IEEE Transactions on Robotics*, 39(2), 234-248.
3. NVIDIA Isaac Team. (2024). *Isaac ROS Navigation Integration Guide*. NVIDIA Technical Report.