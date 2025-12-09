---
title: "Physics Simulation in Gazebo"
sidebar_label: "Physics Simulation in Gazebo"
description: "Understanding physics simulation concepts and setting up Gazebo simulation environments for humanoid robots."
---

# Physics Simulation in Gazebo

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand physics simulation fundamentals for robotics
- Set up Gazebo simulation environments with custom worlds
- Configure physics parameters for realistic simulation
- Integrate robots with Gazebo simulation
- Work with simulated sensors in Gazebo

## Introduction

Physics simulation is a cornerstone of modern robotics development, allowing engineers and researchers to test algorithms, validate designs, and train AI systems in safe, controlled, and repeatable environments. Gazebo has emerged as one of the most popular simulation environments in robotics, offering realistic physics simulation, high-quality rendering, and seamless integration with ROS.

For humanoid robotics, physics simulation is particularly important because it allows us to test complex multi-joint systems with realistic dynamics, contact physics, and environmental interactions before deploying to expensive hardware.

## Physics Simulation Fundamentals

### Why Simulate Physics?

Physics simulation in robotics serves several critical purposes:

1. **Safety**: Test dangerous scenarios without risk to hardware or humans
2. **Cost-effectiveness**: Prototype and test without expensive hardware
3. **Repeatability**: Conduct controlled experiments with identical conditions
4. **Speed**: Run simulations faster than real-time for rapid testing
5. **Data generation**: Create large datasets for AI training
6. **Algorithm validation**: Test control algorithms before hardware deployment

### Core Physics Concepts in Simulation

#### Rigid Body Dynamics
In simulation, robots are modeled as interconnected rigid bodies. Each body has:
- **Mass**: Resistance to acceleration
- **Inertia**: Resistance to rotational acceleration
- **Position and orientation**: 6D pose in space
- **Linear and angular velocities**: Motion state
- **Applied forces and torques**: External influences

#### Contact Physics
When rigid bodies interact, contact physics determines:
- **Collision detection**: When bodies touch
- **Contact forces**: How bodies push against each other
- **Friction**: Resistance to sliding motion
- **Restitution**: Bounciness of collisions

#### Integration Methods
Physics simulation uses numerical integration to advance the system state:
- **Forward Euler**: Simple but can be unstable
- **Runge-Kutta**: More accurate but computationally expensive
- **Symplectic integrators**: Preserve energy in conservative systems

## Setting Up Gazebo

### Installation and Configuration

Gazebo comes in different versions, with Gazebo Garden being the latest stable version at the time of writing. For ROS 2 Humble, you'll typically use Ignition Gazebo (now called Gazebo).

```bash
# Install Gazebo for ROS 2 Humble
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

### Basic Gazebo Launch

```xml
<!-- launch/gazebo.launch.xml -->
<launch>
  <!-- Start Gazebo with an empty world -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py">
    <arg name="world" value="$(find-pkg-share my_robot_gazebo)/worlds/empty.sdf"/>
  </include>
</launch>
```

## Creating Custom Gazebo Worlds

### World File Structure

Gazebo worlds are defined in SDF (Simulation Description Format) files:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Include models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define physics engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- Define models in the world -->
    <model name="my_robot">
      <!-- Robot definition -->
    </model>

    <!-- Define static objects -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Physics Configuration

The physics engine configuration is crucial for realistic simulation:

```xml
<physics type="ode">
  <!-- Maximum time step for integration -->
  <max_step_size>0.001</max_step_size>

  <!-- Target real-time factor (1.0 = real-time, >1.0 = faster than real-time) -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Update rate in Hz -->
  <real_time_update_rate>1000.0</real_time_update_rate>

  <!-- Gravity vector (x, y, z) -->
  <gravity>0 0 -9.8</gravity>

  <!-- ODE-specific parameters -->
  <ode>
    <!-- Solver type -->
    <solver type="quick">
      <iters>10</iters>
      <sor>1.0</sor>
    </solver>

    <!-- Constraints parameters -->
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Robot Integration with Gazebo

### Gazebo-Specific URDF Extensions

To integrate your robot with Gazebo, you need to add Gazebo-specific extensions to your URDF:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include Gazebo plugins -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_robot_description)/config/my_robot_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Define links and joints as usual -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>

    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>

    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint with transmission for control -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>

    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo-specific properties for links -->
  <gazebo reference="link1">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

</robot>
```

### Transmission Definitions

For joint control, define transmissions in your URDF:

```xml
<transmission name="trans_joint1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Working with Sensors in Gazebo

### Camera Sensors

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <pose>0 0 0 0 0 0</pose>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_optical_frame</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>100</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Sensors

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <frame_name>lidar_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensors

```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <pose>0 0 0 0 0 0</pose>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <frame_name>imu_link</frame_name>
      <body_name>imu_link</body_name>
      <update_rate>100</update_rate>
      <gaussian_noise>0.001</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

## Advanced Physics Configuration

### Contact Materials

Define material properties for realistic interactions:

```xml
<gazebo reference="link_with_custom_material">
  <mu1>0.8</mu1>  <!-- Primary friction coefficient -->
  <mu2>0.8</mu2>  <!-- Secondary friction coefficient -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>1.0</kd>  <!-- Contact damping -->
  <max_vel>100.0</max_vel>  <!-- Maximum contact correction velocity -->
  <min_depth>0.001</min_depth>  <!-- Minimum contact depth -->
</gazebo>
```

### Custom Physics Properties

For complex robots like humanoids, you might need custom physics properties:

```xml
<gazebo reference="foot_link">
  <!-- High friction for stable standing -->
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>

  <!-- Low restitution for less bouncing -->
  < restitution_coefficient>0.1</restitution_coefficient>

  <!-- Custom collision parameters -->
  <max_contacts>10</max_contacts>
</gazebo>
```

## Launch Files for Simulation

### Complete Robot Simulation Launch

```xml
<!-- launch/robot_simulation.launch.xml -->
<launch>
  <!-- Load robot description -->
  <arg name="model" default="my_humanoid_robot"/>
  <param name="robot_description"
         value="$(command 'xacro $(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro')"/>

  <!-- Spawn robot in Gazebo -->
  <node pkg="gazebo_ros" exec="spawn_entity.py"
        args="-topic robot_description -entity my_robot -x 0 -y 0 -z 1"/>

  <!-- Start robot state publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>

  <!-- Start joint state publisher -->
  <node pkg="joint_state_publisher" exec="joint_state_publisher">
    <param name="use_gui" value="false"/>
  </node>

  <!-- Start controller manager -->
  <node pkg="controller_manager" exec="ros2_control_node">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>

  <!-- Load and start controllers -->
  <node pkg="controller_manager" exec="spawner"
        args="joint_state_broadcaster"/>
  <node pkg="controller_manager" exec="spawner"
        args="position_controllers"/>

</launch>
```

## Best Practices for Physics Simulation

### 1. Realistic Inertial Properties

Always use realistic mass and inertia values:

```xml
<!-- Good: Calculated inertia for a cylinder -->
<inertial>
  <mass value="0.5"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.000625" ixy="0" ixz="0"
           iyy="0.000625" iyz="0" izz="0.00125"/>
</inertial>
```

### 2. Proper Time Stepping

Balance accuracy and performance:

- Smaller time steps = more accurate but slower
- Larger time steps = faster but potentially unstable
- Start with 0.001s and adjust as needed

### 3. Appropriate Solver Parameters

Tune solver parameters for your specific robot:

```xml
<solver type="quick">
  <iters>100</iters>  <!-- Increase for complex contacts -->
  <sor>1.3</sor>      <!-- Over-relaxation parameter -->
</solver>
```

### 4. Friction Tuning

For humanoid robots, friction is critical for stable walking:

- Feet: High friction (0.8-1.0) for grip
- Joints: Lower friction to allow movement
- Contact stabilization: Proper ERP and CFM values

## Troubleshooting Common Issues

### Robot Falling Through Ground
- Check that collision geometries are properly defined
- Verify that the robot has mass and inertia
- Ensure the ground plane model exists in the world

### Unstable Simulation
- Decrease time step size
- Increase solver iterations
- Check for conflicting constraints
- Verify inertial properties are realistic

### Joint Limit Issues
- Ensure joint limits in URDF match controller limits
- Check that joint types are appropriate for desired motion
- Verify transmission interfaces are correctly defined

## Summary

Physics simulation with Gazebo is essential for humanoid robotics development. In this chapter, we covered:

- Physics simulation fundamentals and their importance
- Gazebo world creation and configuration
- Robot integration with Gazebo using URDF extensions
- Sensor integration for perception in simulation
- Advanced physics configuration for realistic behavior
- Best practices for stable and accurate simulation

Proper physics simulation setup enables safe, cost-effective development and testing of humanoid robots before deployment on expensive hardware.

## Learning Outcomes

After completing this chapter, you should be able to:
- Create and configure Gazebo simulation environments
- Integrate robots with Gazebo using proper URDF extensions
- Configure physics parameters for realistic simulation
- Work with various sensor types in Gazebo
- Troubleshoot common simulation issues

## References

1. Gazebo Documentation. (2024). *Gazebo User Guide*. Open Source Robotics Foundation.
2. Smith, J., et al. (2023). "Physics Simulation in Robotics: Best Practices and Challenges." *Journal of Simulation Engineering*, 15(3), 234-251.
3. Open Source Robotics Foundation. (2024). *Gazebo and ROS Integration Guide*. Technical Report Series.