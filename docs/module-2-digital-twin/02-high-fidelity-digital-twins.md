---
title: "High-Fidelity Digital Twins in Unity"
sidebar_label: "High-Fidelity Digital Twins in Unity"
description: "Learning to create high-fidelity robot models in Unity for Human-Robot Interaction (HRI) and visual realism."
---

# High-Fidelity Digital Twins in Unity

## Learning Objectives

By the end of this chapter, you will be able to:
- Create high-fidelity robot models in Unity for realistic visualization
- Understand Human-Robot Interaction (HRI) interfaces in Unity
- Integrate Unity with robotics workflows using ROS# or similar bridges
- Implement realistic physics and rendering for digital twins
- Create interactive 3D environments for robot simulation

## Introduction

High-fidelity digital twins are essential for creating realistic simulations that bridge the gap between virtual and physical robotics. Unity, with its powerful rendering engine and physics simulation, provides an excellent platform for creating detailed, interactive digital twins of robots and their environments.

For humanoid robotics, Unity enables the creation of photorealistic environments where human-robot interaction can be tested and refined before deployment on physical robots. The platform's real-time rendering capabilities and extensive asset ecosystem make it ideal for creating immersive simulation experiences.

## Unity for Robotics Overview

### Why Unity for Robotics?

Unity offers several advantages for robotics simulation:

- **Photorealistic Rendering**: Advanced lighting, shadows, and materials
- **Real-time Performance**: Interactive simulation at high frame rates
- **Asset Ecosystem**: Extensive library of 3D models and environments
- **Cross-platform**: Deploy to multiple platforms including VR/AR
- **Scripting**: C# scripting for custom robot behaviors
- **Physics**: Realistic physics simulation with PhysX engine

### Unity Robotics Ecosystem

Unity provides several tools for robotics:

- **Unity Robotics Hub**: Centralized access to robotics packages
- **ROS#**: ROS/ROS2 bridge for Unity
- **Unity ML-Agents**: Reinforcement learning framework
- **Unity Perception**: Synthetic data generation tools
- **Oculus XR**: VR integration for immersive HRI

## Setting Up Unity for Robotics

### Installation and Setup

To get started with Unity for robotics:

1. **Install Unity Hub**: Download from unity.com
2. **Install Unity Editor**: Version 2021.3 LTS or later recommended
3. **Install Robotics Packages**: Through Unity Package Manager
4. **Set up ROS Bridge**: Install ROS# or similar bridge

### Essential Unity Concepts for Robotics

#### GameObjects and Components
```csharp
using UnityEngine;

public class RobotController : MonoBehaviour
{
    // Robot components as GameObjects
    public Transform head;
    public Transform[] joints;
    public Transform[] sensors;

    // Robot properties
    public float maxSpeed = 1.0f;
    public float maxAngularSpeed = 1.0f;

    void Start()
    {
        InitializeRobot();
    }

    void InitializeRobot()
    {
        // Initialize robot components
        foreach (Transform joint in joints)
        {
            // Configure joint properties
            ConfigurableJoint configJoint = joint.GetComponent<ConfigurableJoint>();
            if (configJoint != null)
            {
                // Set joint limits and properties
                configJoint.linearLimit = new SoftJointLimit { limit = 0.1f };
            }
        }
    }
}
```

#### Physics and Colliders
```csharp
using UnityEngine;

public class RobotPhysics : MonoBehaviour
{
    public Rigidbody[] robotLinks;
    public float massScale = 1.0f;

    void Start()
    {
        ConfigurePhysics();
    }

    void ConfigurePhysics()
    {
        foreach (Rigidbody link in robotLinks)
        {
            // Set mass based on real-world values
            link.mass = link.mass * massScale;

            // Configure collision properties
            link.drag = 0.1f;
            link.angularDrag = 0.05f;
        }
    }
}
```

## Creating High-Fidelity Robot Models

### 3D Modeling Principles

For realistic robot models in Unity:

1. **Proper Scaling**: Use real-world units (meters)
2. **Accurate Dimensions**: Match physical robot specifications
3. **Appropriate Detail**: Balance visual quality with performance
4. **Correct Hierarchy**: Maintain proper parent-child relationships

### Robot Model Structure

```csharp
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class RobotLink : MonoBehaviour
{
    [Header("Link Properties")]
    public float mass = 1.0f;
    public Vector3 centerOfMass = Vector3.zero;
    public PhysicMaterial physicMaterial;

    [Header("Visual Properties")]
    public Material defaultMaterial;
    public Material highlightMaterial;

    [Header("Joint Configuration")]
    public Joint connectedJoint;
    public float jointLimit = 90f;

    void Start()
    {
        ConfigureRigidbody();
        ConfigureColliders();
    }

    void ConfigureRigidbody()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.mass = mass;
        rb.centerOfMass = centerOfMass;
    }

    void ConfigureColliders()
    {
        Collider[] colliders = GetComponents<Collider>();
        foreach (Collider col in colliders)
        {
            col.material = physicMaterial;
        }
    }

    public void Highlight()
    {
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        foreach (Renderer rend in renderers)
        {
            rend.material = highlightMaterial;
        }
    }

    public void ResetAppearance()
    {
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        foreach (Renderer rend in renderers)
        {
            rend.material = defaultMaterial;
        }
    }
}
```

### Advanced Materials and Shaders

For photorealistic appearance:

```csharp
using UnityEngine;

public class RobotMaterialController : MonoBehaviour
{
    [Header("Material Properties")]
    public Material[] robotMaterials;
    public float metallicValue = 0.8f;
    public float smoothnessValue = 0.6f;
    public Texture2D normalMap;
    public Texture2D roughnessMap;

    void Start()
    {
        ConfigureMaterials();
    }

    void ConfigureMaterials()
    {
        foreach (Material mat in robotMaterials)
        {
            if (mat != null)
            {
                mat.SetFloat("_Metallic", metallicValue);
                mat.SetFloat("_Smoothness", smoothnessValue);

                if (normalMap != null)
                    mat.SetTexture("_BumpMap", normalMap);

                if (roughnessMap != null)
                    mat.SetTexture("_MetallicGlossMap", roughnessMap);
            }
        }
    }

    public void SetMaterialProperty(string propertyName, float value)
    {
        foreach (Material mat in robotMaterials)
        {
            if (mat != null)
            {
                mat.SetFloat(propertyName, value);
            }
        }
    }
}
```

## Human-Robot Interaction (HRI) Interfaces

### Creating Interactive Interfaces

Unity excels at creating intuitive HRI interfaces:

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class HRIInterface : MonoBehaviour
{
    [Header("UI Elements")]
    public Button[] commandButtons;
    public Slider speedSlider;
    public TMP_InputField commandInput;
    public TextMeshProUGUI statusText;
    public Image progressBar;

    [Header("Robot Communication")]
    public RobotController robotController;

    void Start()
    {
        InitializeUI();
    }

    void InitializeUI()
    {
        // Setup command buttons
        foreach (Button btn in commandButtons)
        {
            btn.onClick.AddListener(() => ProcessCommand(btn.name));
        }

        // Setup slider for speed control
        speedSlider.onValueChanged.AddListener(OnSpeedChanged);

        // Setup command input
        commandInput.onSubmit.AddListener(ProcessTextInput);
    }

    void ProcessCommand(string command)
    {
        // Send command to robot
        if (robotController != null)
        {
            robotController.ExecuteCommand(command);
            UpdateStatus($"Command sent: {command}");
        }
    }

    void OnSpeedChanged(float speed)
    {
        if (robotController != null)
        {
            robotController.SetSpeed(speed);
        }
    }

    void ProcessTextInput(string text)
    {
        ProcessCommand(text);
        commandInput.text = "";
    }

    void UpdateStatus(string message)
    {
        statusText.text = message;
    }

    public void UpdateProgress(float progress)
    {
        progressBar.fillAmount = progress;
    }
}
```

### VR/AR Integration for HRI

```csharp
using UnityEngine;
using UnityEngine.XR;

public class VRHRIController : MonoBehaviour
{
    [Header("VR Controllers")]
    public XRNode leftController;
    public XRNode rightController;

    [Header("Interaction Objects")]
    public GameObject[] interactableObjects;
    public LayerMask interactionLayer;

    void Update()
    {
        HandleVRInput();
        UpdateInteraction();
    }

    void HandleVRInput()
    {
        // Get controller positions and rotations
        Vector3 leftPos = InputTracking.GetLocalPosition(leftController);
        Quaternion leftRot = InputTracking.GetLocalRotation(leftController);

        Vector3 rightPos = InputTracking.GetLocalPosition(rightController);
        Quaternion rightRot = InputTracking.GetLocalRotation(rightController);

        // Update controller visual representations
        // This would typically be handled by XR Interaction Toolkit
    }

    void UpdateInteraction()
    {
        // Check for raycast interactions
        if (Physics.Raycast(Camera.main.transform.position, Camera.main.transform.forward,
                           out RaycastHit hit, 10f, interactionLayer))
        {
            HighlightInteractable(hit.collider.gameObject);
        }
    }

    void HighlightInteractable(GameObject obj)
    {
        // Highlight the object being looked at
        Renderer rend = obj.GetComponent<Renderer>();
        if (rend != null)
        {
            rend.material.color = Color.yellow;
        }
    }
}
```

## ROS Integration with Unity

### Setting up ROS# Bridge

Unity Robotics provides the ROS# package for ROS communication:

```csharp
using System.Collections;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class UnityROSConnector : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosBridgeServerUrl = "ws://192.168.1.10:9090";
    public int timeout = 10;

    [Header("Topics")]
    public string jointStatesTopic = "/joint_states";
    public string cmdVelTopic = "/cmd_vel";
    public string imageTopic = "/camera/image_raw";

    private RosSocket rosSocket;

    void Start()
    {
        ConnectToROS();
    }

    void ConnectToROS()
    {
        RosBridgeClient.Protocols.WebSocketNetProtocol protocol =
            new RosBridgeClient.Protocols.WebSocketNetProtocol(rosBridgeServerUrl);

        rosSocket = new RosSocket(protocol, OnConnected, OnClosed, OnError);

        // Wait for connection
        StartCoroutine(WaitForConnection());
    }

    IEnumerator WaitForConnection()
    {
        float startTime = Time.time;
        while (rosSocket == null || !rosSocket.protocol.IsConnected)
        {
            if (Time.time - startTime > timeout)
            {
                Debug.LogError("Failed to connect to ROS bridge");
                yield break;
            }
            yield return new WaitForSeconds(0.1f);
        }

        Debug.Log("Connected to ROS bridge successfully");
        SubscribeToTopics();
    }

    void SubscribeToTopics()
    {
        // Subscribe to joint states
        rosSocket.Subscribe<JointState>(jointStatesTopic, OnJointStateReceived);

        // Subscribe to other topics as needed
    }

    void OnJointStateReceived(JointState jointState)
    {
        // Process joint state data
        Debug.Log($"Received joint state with {jointState.name.Length} joints");
    }

    void OnConnected()
    {
        Debug.Log("ROS connection established");
    }

    void OnClosed()
    {
        Debug.Log("ROS connection closed");
    }

    void OnError(string error)
    {
        Debug.LogError($"ROS connection error: {error}");
    }

    void OnDestroy()
    {
        rosSocket?.Close();
    }
}
```

### Publishing Robot Data

```csharp
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

public class RobotDataPublisher : MonoBehaviour
{
    [Header("Publishers")]
    public string tfTopic = "/tf";
    public string odomTopic = "/odom";

    private RosSocket rosSocket;
    private Transform robotTransform;

    void Start()
    {
        robotTransform = transform;
        rosSocket = FindObjectOfType<UnityROSConnector>().rosSocket;
    }

    void Update()
    {
        // Publish robot state at regular intervals
        if (Time.time % 0.1f < Time.deltaTime) // Every 100ms
        {
            PublishRobotState();
        }
    }

    void PublishRobotState()
    {
        // Publish transform
        PublishTransform();

        // Publish odometry
        PublishOdometry();
    }

    void PublishTransform()
    {
        TransformStamped transformStamped = new TransformStamped();
        transformStamped.header.frame_id = "map";
        transformStamped.header.stamp = new TimeData();
        transformStamped.child_frame_id = "robot_base";

        // Set position
        transformStamped.transform.translation.x = robotTransform.position.x;
        transformStamped.transform.translation.y = robotTransform.position.y;
        transformStamped.transform.translation.z = robotTransform.position.z;

        // Set rotation
        transformStamped.transform.rotation.x = robotTransform.rotation.x;
        transformStamped.transform.rotation.y = robotTransform.rotation.y;
        transformStamped.transform.rotation.z = robotTransform.rotation.z;
        transformStamped.transform.rotation.w = robotTransform.rotation.w;

        rosSocket.Publish(tfTopic, transformStamped);
    }

    void PublishOdometry()
    {
        Odometry odom = new Odometry();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Set pose
        odom.pose.pose.position.x = robotTransform.position.x;
        odom.pose.pose.position.y = robotTransform.position.y;
        odom.pose.pose.position.z = robotTransform.position.z;

        odom.pose.pose.orientation.x = robotTransform.rotation.x;
        odom.pose.pose.orientation.y = robotTransform.rotation.y;
        odom.pose.pose.orientation.z = robotTransform.rotation.z;
        odom.pose.pose.orientation.w = robotTransform.rotation.w;

        rosSocket.Publish(odomTopic, odom);
    }
}
```

## Advanced Simulation Features

### Physics Simulation Enhancement

```csharp
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class AdvancedRobotPhysics : MonoBehaviour
{
    [Header("Physics Configuration")]
    public float groundFriction = 0.8f;
    public float airDrag = 0.01f;
    public float angularDrag = 0.05f;
    public float collisionThreshold = 0.1f;

    [Header("Balance Control")]
    public Transform centerOfMass;
    public float balanceThreshold = 10f;

    private Rigidbody rb;
    private bool isOnGround = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (centerOfMass != null)
        {
            rb.centerOfMass = centerOfMass.localPosition;
        }
    }

    void FixedUpdate()
    {
        UpdatePhysics();
        CheckBalance();
    }

    void UpdatePhysics()
    {
        // Apply ground-specific physics
        if (isOnGround)
        {
            rb.drag = groundFriction;
        }
        else
        {
            rb.drag = airDrag;
        }

        rb.angularDrag = angularDrag;
    }

    void CheckBalance()
    {
        // Check if robot is in danger of falling
        Vector3 upVector = transform.up;
        float angle = Vector3.Angle(upVector, Vector3.up);

        if (angle > balanceThreshold)
        {
            // Attempt to restore balance
            RestoreBalance();
        }
    }

    void RestoreBalance()
    {
        // Simple balance restoration - in reality, this would involve
        // complex control algorithms
        Vector3 targetUp = Vector3.up;
        Quaternion targetRotation = Quaternion.FromToRotation(transform.up, targetUp) * transform.rotation;

        rb.MoveRotation(Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 2f));
    }

    void OnCollisionEnter(Collision collision)
    {
        isOnGround = true;

        // Check collision force
        if (collision.relativeVelocity.magnitude > collisionThreshold)
        {
            HandleCollision(collision);
        }
    }

    void OnCollisionExit(Collision collision)
    {
        isOnGround = false;
    }

    void HandleCollision(Collision collision)
    {
        // Handle significant collisions
        Debug.Log($"Collision detected with {collision.gameObject.name}, force: {collision.relativeVelocity.magnitude}");
    }
}
```

### Realistic Sensor Simulation

```csharp
using UnityEngine;

public class UnitySensorSimulator : MonoBehaviour
{
    [Header("Camera Sensor")]
    public Camera sensorCamera;
    public int imageWidth = 640;
    public int imageHeight = 480;

    [Header("LiDAR Sensor")]
    public float lidarRange = 30f;
    public int lidarResolution = 720;
    public float lidarAngle = 360f;

    [Header("IMU Sensor")]
    public float imuNoise = 0.01f;

    private RenderTexture sensorTexture;
    private float[] lidarData;

    void Start()
    {
        InitializeSensors();
    }

    void InitializeSensors()
    {
        // Setup camera sensor
        if (sensorCamera != null)
        {
            sensorTexture = new RenderTexture(imageWidth, imageHeight, 24);
            sensorCamera.targetTexture = sensorTexture;
        }

        // Initialize LiDAR data
        lidarData = new float[lidarResolution];
    }

    void Update()
    {
        if (Time.time % 0.1f < Time.deltaTime) // Every 100ms
        {
            SimulateSensors();
        }
    }

    void SimulateSensors()
    {
        // Simulate camera
        SimulateCamera();

        // Simulate LiDAR
        SimulateLiDAR();

        // Simulate IMU
        SimulateIMU();
    }

    void SimulateCamera()
    {
        if (sensorCamera != null)
        {
            // Render the scene
            sensorCamera.Render();

            // The image is now available in sensorTexture
            // This would be processed and sent to ROS
        }
    }

    void SimulateLiDAR()
    {
        for (int i = 0; i < lidarResolution; i++)
        {
            float angle = (i * lidarAngle / lidarResolution) * Mathf.Deg2Rad;
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));

            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, lidarRange))
            {
                lidarData[i] = hit.distance + Random.Range(-imuNoise, imuNoise);
            }
            else
            {
                lidarData[i] = lidarRange;
            }
        }
    }

    void SimulateIMU()
    {
        // Simulate IMU data with noise
        Vector3 linearAcceleration = rb.velocity / Time.deltaTime + Random.insideUnitSphere * imuNoise;
        Vector3 angularVelocity = rb.angularVelocity + Random.insideUnitSphere * imuNoise;

        // This data would be published to ROS
    }
}
```

## Performance Optimization

### Level of Detail (LOD) System

```csharp
using UnityEngine;

public class RobotLODSystem : MonoBehaviour
{
    [Header("LOD Configuration")]
    public float[] lodDistances = { 10f, 30f, 60f };
    public GameObject[] lodMeshes;

    private int currentLOD = 0;

    void Start()
    {
        InitializeLOD();
    }

    void Update()
    {
        UpdateLOD();
    }

    void InitializeLOD()
    {
        // Hide all LOD meshes initially
        foreach (GameObject lodMesh in lodMeshes)
        {
            if (lodMesh != null)
                lodMesh.SetActive(false);
        }

        // Show highest detail LOD initially
        if (lodMeshes.Length > 0 && lodMeshes[0] != null)
            lodMeshes[0].SetActive(true);
    }

    void UpdateLOD()
    {
        float distanceToCamera = Vector3.Distance(transform.position, Camera.main.transform.position);

        int newLOD = lodDistances.Length; // Default to lowest detail

        for (int i = 0; i < lodDistances.Length; i++)
        {
            if (distanceToCamera < lodDistances[i])
            {
                newLOD = i;
                break;
            }
        }

        if (newLOD != currentLOD)
        {
            // Switch LOD
            if (currentLOD < lodMeshes.Length && lodMeshes[currentLOD] != null)
                lodMeshes[currentLOD].SetActive(false);

            if (newLOD < lodMeshes.Length && lodMeshes[newLOD] != null)
                lodMeshes[newLOD].SetActive(true);

            currentLOD = newLOD;
        }
    }
}
```

### Occlusion Culling for Large Environments

```csharp
using UnityEngine;

public class EnvironmentOcclusionManager : MonoBehaviour
{
    [Header("Occlusion Settings")]
    public GameObject[] environmentObjects;
    public float cullDistance = 100f;

    void Update()
    {
        CullDistantObjects();
    }

    void CullDistantObjects()
    {
        foreach (GameObject obj in environmentObjects)
        {
            if (obj != null)
            {
                float distance = Vector3.Distance(Camera.main.transform.position, obj.transform.position);
                obj.SetActive(distance < cullDistance);
            }
        }
    }
}
```

## Best Practices for Digital Twins

### 1. Realistic Physics Parameters

Always use realistic physical parameters for accurate simulation:

```csharp
// Good: Realistic parameters
public class RealisticRobotParameters : MonoBehaviour
{
    public float robotMass = 50f; // 50kg for a small humanoid
    public float linkMassScale = 1f;
    public float frictionCoefficient = 0.7f; // Rubber-like friction
    public float restitution = 0.1f; // Low bounce
}
```

### 2. Proper Scaling

Maintain 1:1 scaling between virtual and real worlds:

```csharp
public class ScaleValidator : MonoBehaviour
{
    public float realWorldScale = 1f; // Meters

    void ValidateScale()
    {
        // Ensure all robot dimensions match real-world values
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        foreach (Renderer rend in renderers)
        {
            Bounds bounds = rend.bounds;
            Debug.Log($"Object bounds: {bounds.size}");
            // Compare with real-world dimensions
        }
    }
}
```

### 3. Efficient Resource Management

```csharp
using UnityEngine;
using System.Collections;

public class EfficientResourceManager : MonoBehaviour
{
    [Header("Resource Management")]
    public int maxTextureResolution = 2048;
    public int maxMeshVertices = 10000;

    void Start()
    {
        OptimizeResources();
    }

    void OptimizeResources()
    {
        // Optimize textures
        Texture[] textures = Resources.FindObjectsOfTypeAll<Texture>();
        foreach (Texture tex in textures)
        {
            if (tex.width > maxTextureResolution || tex.height > maxTextureResolution)
            {
                // Use texture compression or lower resolution
                Debug.LogWarning($"Texture {tex.name} exceeds max resolution");
            }
        }

        // Optimize meshes
        MeshFilter[] meshFilters = GetComponentsInChildren<MeshFilter>();
        foreach (MeshFilter filter in meshFilters)
        {
            if (filter.sharedMesh.vertexCount > maxMeshVertices)
            {
                // Use LOD or simplify mesh
                Debug.LogWarning($"Mesh {filter.name} exceeds max vertices");
            }
        }
    }
}
```

## Integration with External Tools

### Blender to Unity Pipeline

For importing robot models from Blender:

1. **Export Settings**: Use FBX format with proper scale (1m = 1 unit)
2. **Coordinate System**: Ensure Y-up to Y-up conversion
3. **Materials**: Use Universal Render Pipeline (URP) or High Definition Render Pipeline (HDRP)
4. **Animation**: Export with proper bone weights and animations

### CAD to Unity Workflow

For importing CAD models:

1. **Export from CAD**: STL, OBJ, or STEP format
2. **Import to Blender**: Convert and optimize
3. **Export to Unity**: FBX format
4. **Setup in Unity**: Add colliders, materials, and physics properties

## Troubleshooting Common Issues

### 1. Physics Instability
- **Problem**: Robot falls through floor or behaves erratically
- **Solution**: Check mass distribution, increase solver iterations, reduce time step

### 2. Performance Issues
- **Problem**: Low frame rate in complex scenes
- **Solution**: Implement LOD, use occlusion culling, optimize draw calls

### 3. ROS Connection Problems
- **Problem**: Cannot connect to ROS bridge
- **Solution**: Check network settings, firewall, ROS bridge status

### 4. Scale Mismatch
- **Problem**: Robot appears too large or small
- **Solution**: Verify units (Unity uses meters), check import scale settings

## Summary

High-fidelity digital twins in Unity provide powerful capabilities for robotics development. In this chapter, we covered:

- Unity's advantages for robotics simulation and HRI
- Creating realistic robot models with proper physics
- Human-robot interaction interfaces in Unity
- ROS integration using Unity Robotics packages
- Advanced simulation features for realistic behavior
- Performance optimization techniques
- Best practices for digital twin development

Unity's combination of realistic rendering, physics simulation, and interactive capabilities makes it an excellent platform for developing and testing humanoid robots before deployment on physical hardware.

## Learning Outcomes

After completing this chapter, you should be able to:
- Create high-fidelity robot models in Unity with realistic physics
- Design intuitive Human-Robot Interaction interfaces
- Integrate Unity with ROS for complete robotics workflows
- Implement advanced simulation features like sensors and balance control
- Optimize Unity scenes for performance in robotics applications
- Troubleshoot common issues in Unity robotics projects

## References

1. Unity Technologies. (2024). *Unity Robotics Integration Guide*. Unity Technologies.
2. Anderson, M., et al. (2023). "Digital Twins in Robotics: A Survey." *IEEE Robotics & Automation Magazine*, 30(1), 45-58.
3. Unity Robotics Team. (2024). *ROS# Bridge Documentation*. Unity Robotics Hub.