# Unity Visualization Examples for Physical AI

This directory contains Unity examples for Physical AI visualization as referenced in the textbook. Since Unity is a visual environment that can't run directly from command line, this document provides the concepts and C# scripts that would be used in the Unity project.

## Overview

Unity provides an excellent platform for creating intuitive visualizations of Physical AI systems. It allows for:
- High-quality 3D rendering of robots and environments
- Interactive interfaces for human-robot interaction
- VR/AR capabilities for immersive control
- Realistic simulations for synthetic data generation

## Basic Unity Project Structure for Physical AI

### 1. Scene Setup

A basic Physical AI Unity scene would contain:

- **Main Camera**: For viewing the scene
- **Robot Model**: 3D representation of the robot
- **Environment**: A 3D environment where the robot operates
- **Lighting**: Proper illumination for realistic rendering
- **UI Canvas**: For displaying information and controls

### 2. Robot Control Script

Here's a C# script that would handle robot control in Unity:

```csharp
using UnityEngine;
using System.Collections;

public class RobotController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 2.0f;
    public float turnSpeed = 90.0f;  // degrees per second

    [Header("ROS Connection")]
    public string rosBridgeUrl = "ws://localhost:9090";

    private float targetVelocity = 0f;
    private float targetAngularVelocity = 0f;
    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            // Add rigidbody if not present
            rb = gameObject.AddComponent<Rigidbody>();
            rb.useGravity = false;
            rb.constraints = RigidbodyConstraints.FreezeRotationX | 
                            RigidbodyConstraints.FreezeRotationZ |
                            RigidbodyConstraints.FreezePositionY;
        }
    }

    void Update()
    {
        // Apply movement based on target velocities
        if (rb != null)
        {
            // For differential drive robots
            transform.Translate(Vector3.forward * targetVelocity * Time.deltaTime);
            transform.Rotate(Vector3.up, targetAngularVelocity * Time.deltaTime);
        }
    }

    // Methods to be called via ROS bridge or other interfaces
    public void SetVelocity(float linear, float angular)
    {
        targetVelocity = linear;
        targetAngularVelocity = angular;
    }

    // Accessors for sensor simulation
    public Vector3 GetPosition()
    {
        return transform.position;
    }

    public Quaternion GetOrientation()
    {
        return transform.rotation;
    }
}
```

### 3. Camera and Sensor Simulation

Unity can simulate various sensors used in Physical AI systems:

```csharp
using UnityEngine;

public class CameraSensor : MonoBehaviour
{
    [Header("Camera Settings")]
    public int imageWidth = 640;
    public int imageHeight = 480;
    public Camera cam;

    private RenderTexture renderTexture;

    void Start()
    {
        if (cam == null)
            cam = GetComponent<Camera>();
            
        // Create a render texture for the camera
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        cam.targetTexture = renderTexture;
    }

    // Method to capture and potentially send image data
    public Texture2D CaptureImage()
    {
        RenderTexture.active = renderTexture;
        Texture2D image = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        image.Apply();
        RenderTexture.active = null;
        
        return image;
    }
}
```

### 4. ROS Bridge Connection

For connecting Unity to ROS 2, you would typically use a ROS bridge. Here's an example of how to interface with a WebSocket ROS bridge:

```csharp
using UnityEngine;
using System.Collections;
using RosSharp.RosBridgeClient;

public class UnityRosBridge : MonoBehaviour
{
    [Header("ROS Settings")]
    public string rosBridgeServerUrl = "ws://192.168.1.10:9090";
    
    private RosSocket rosSocket;

    void Start()
    {
        ConnectToRosBridge();
    }

    void ConnectToRosBridge()
    {
        RosBridgeClient.Protocols.WebSocketNetProtocol webSocket = 
            new RosBridgeClient.Protocols.WebSocketNetProtocol(rosBridgeServerUrl);
            
        rosSocket = new RosSocket(webSocket, new RosBridgeClient.MessageTypes.Std_msgs.StringMsg.Deserializer());
        
        Debug.Log("Connected to ROS Bridge at: " + rosBridgeServerUrl);
    }

    // Example method to publish a message
    public void PublishMessage(string topic, string message)
    {
        if (rosSocket != null)
        {
            RosBridgeClient.MessageTypes.Std_msgs.StringMsg stringMsg = 
                new RosBridgeClient.MessageTypes.Std_msgs.StringMsg(message);
                
            rosSocket.Publish(topic, stringMsg);
        }
    }

    // Example method to subscribe to a topic
    public void SubscribeToTopic(string topic)
    {
        if (rosSocket != null)
        {
            rosSocket.Subscribe<RosBridgeClient.MessageTypes.Std_msgs.StringMsg>(
                topic, 
                ProcessMessage
            );
        }
    }

    private void ProcessMessage(RosBridgeClient.MessageTypes.Std_msgs.StringMsg message)
    {
        Debug.Log("Received message: " + message.data);
        // Process the received message as needed
    }

    void OnApplicationQuit()
    {
        if (rosSocket != null)
        {
            rosSocket.Close();
        }
    }
}
```

### 5. User Interface for VLA Interaction

Unity provides excellent tools for creating intuitive user interfaces, which is important for Vision-Language-Action systems:

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class VlaInterface : MonoBehaviour
{
    [Header("UI Elements")]
    public TMP_InputField commandInput;
    public Button sendButton;
    public TMP_Text responseText;
    public TMP_Text statusText;

    [Header("Robot Controller")]
    public RobotController robot;

    void Start()
    {
        sendButton.onClick.AddListener(ProcessCommand);
        commandInput.onSubmit.AddListener(delegate { ProcessCommand(); });
    }

    void ProcessCommand()
    {
        string command = commandInput.text;
        if (string.IsNullOrEmpty(command))
            return;

        statusText.text = "Processing command...";
        
        // In a real implementation, send this command to your VLA system
        // For now, we'll simulate a response
        StartCoroutine(SimulateVlaResponse(command));
    }

    IEnumerator SimulateVlaResponse(string command)
    {
        // Simulate processing time
        yield return new WaitForSeconds(1.0f);
        
        // Generate a mock response based on the command
        string response = GenerateMockResponse(command);
        responseText.text = response;
        statusText.text = "Ready";
        
        // Clear input field
        commandInput.text = "";
    }

    string GenerateMockResponse(string command)
    {
        if (command.ToLower().Contains("move") || command.ToLower().Contains("go"))
        {
            // Extract direction if any
            if (command.ToLower().Contains("forward"))
            {
                robot.SetVelocity(0.5f, 0f);  // Move forward
                return "Moving forward as requested.";
            }
            else if (command.ToLower().Contains("backward"))
            {
                robot.SetVelocity(-0.5f, 0f);  // Move backward
                return "Moving backward as requested.";
            }
            else if (command.ToLower().Contains("left"))
            {
                robot.SetVelocity(0f, 45f * Mathf.Deg2Rad);  // Turn left
                return "Turning left as requested.";
            }
            else if (command.ToLower().Contains("right"))
            {
                robot.SetVelocity(0f, -45f * Mathf.Deg2Rad);  // Turn right
                return "Turning right as requested.";
            }
            else
            {
                robot.SetVelocity(0.5f, 0f);  // Default forward movement
                return "I will move in the specified direction.";
            }
        }
        else
        {
            return "I received your command: '" + command + "'. I'm not sure how to execute it yet.";
        }
    }
}
```

## Setting Up a Physical AI Unity Project

### 1. Install Required Packages
- Unity 2022.3 LTS or newer
- ROS# package for ROS bridge communication
- TextMeshPro for UI text elements

### 2. Import Robot Models
- Import 3D models of your robot
- Set up colliders for physics simulation
- Configure materials and textures for realistic appearance

### 3. Configure Environment
- Create or import environment assets
- Set up lighting that matches your real-world environment
- Add obstacles and objects for interaction

### 4. Connect to Gazebo/Isaac Sim
- Use the ROS bridge to connect Unity visualization to Gazebo or Isaac Sim
- Synchronize robot state between simulation and Unity visualization
- Ensure visual representation matches physical simulation

## Best Practices for Physical AI Visualization

1. **Accurate Representation**: Ensure the Unity model accurately represents the physical robot
2. **Low Latency**: Minimize delay between physical robot/simulation and Unity visualization
3. **Intuitive UI**: Create interfaces that are easy for humans to understand and use
4. **Performance**: Optimize rendering for smooth visualization even on standard hardware
5. **Sync**: Maintain synchronization between simulation and visualization

## Integration with the Rest of the Physical AI System

Unity visualization would typically be part of a larger Physical AI system that includes:
- Gazebo for physics simulation
- Isaac Sim for high-fidelity perception simulation
- ROS 2 for message passing
- VLA systems for natural interaction

The Unity component would primarily serve as the visualization layer, while computation happens in other parts of the system.

## Example Scene Hierarchy

```
- PhysicalAI_Scene
  - Main Camera
  - Directional Light
  - Robot (with RobotController.cs)
    - Camera (with CameraSensor.cs)
  - Environment
    - Floor
    - Walls
    - Objects
  - UI
    - Canvas
      - CommandInput
      - SendButton
      - ResponseText
      - StatusText
  - UnityRosBridge (empty GameObject with UnityRosBridge.cs)
```

This structure provides a foundation for creating rich visualizations of Physical AI systems in Unity. The actual implementation would involve creating materials, configuring lighting, and fine-tuning the physics properties to match the real robot as closely as possible.