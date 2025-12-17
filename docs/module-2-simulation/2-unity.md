# Unity with ROS-TCP-Connector

Unity is a powerful game engine that can be used for high-fidelity robot simulation with photorealistic rendering and advanced physics. The ROS-TCP-Connector enables seamless communication between Unity and ROS 2.

## Why Unity for Robotics?

Unity offers several advantages for robotics simulation:

- **Photorealistic Graphics**: Advanced rendering for computer vision training
- **Asset Ecosystem**: Vast library of 3D models and environments
- **Cross-Platform**: Deploy to desktop, mobile, VR/AR
- **Machine Learning**: Integration with ML-Agents for reinforcement learning
- **Performance**: Optimized rendering and physics engines

## Prerequisites

- Unity Hub installed (2021.3 LTS or later)
- Unity Editor (download via Unity Hub)
- ROS 2 Humble or later
- Basic C# knowledge (helpful but not required)

## Installation

### Step 1: Install Unity Hub

Download from [Unity Download Page](https://unity.com/download):

```bash
# For Linux
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.AppImage
chmod +x UnityHubSetup.AppImage
./UnityHubSetup.AppImage
```

### Step 2: Install Unity Editor

1. Open Unity Hub
2. Go to **Installs** tab
3. Click **Install Editor**
4. Select Unity 2021.3 LTS or later
5. Add modules:
   - Linux Build Support (if on Linux)
   - Documentation
   - WebGL Build Support (optional)

### Step 3: Create New Project

1. Click **New Project**
2. Select **3D** template
3. Name: `HumanoidRobotSim`
4. Click **Create Project**

### Step 4: Install ROS-TCP-Connector

#### Method 1: Package Manager (Recommended)

1. Open Unity Editor
2. Go to **Window > Package Manager**
3. Click **+** button → **Add package from git URL**
4. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

#### Method 2: Manual Installation

```bash
# Clone repository
cd ~/Downloads
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git

# In Unity:
# Window > Package Manager > + > Add package from disk
# Navigate to: Unity-Robotics-Hub/com.unity.robotics.ros-tcp-connector/package.json
```

### Step 5: Install URDF Importer

```text
# In Package Manager, add:
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

## Setting Up ROS-TCP Endpoint

### On ROS 2 Side

Install the ROS-TCP endpoint:

```bash
# Create workspace if not exists
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone ROS-TCP endpoint
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

Launch the endpoint:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

### On Unity Side

1. Go to **Robotics > ROS Settings**
2. Set **ROS IP Address**: `127.0.0.1`
3. Set **ROS Port**: `10000`
4. Set **Protocol**: `ROS 2`
5. Click **Connect**

## Importing a Humanoid Robot

### Method 1: Import from URDF

1. Prepare your URDF file (e.g., `humanoid.urdf`)
2. In Unity: **Assets > Import Robot from URDF**
3. Select your URDF file
4. Configure import settings:
   - **Axis Type**: Z-Axis
   - **Mesh Decomposer**: VHACD (for collisions)
5. Click **Import**

### Method 2: Build from Scratch

Create a simple humanoid in Unity:

```csharp
// HumanoidBuilder.cs
using UnityEngine;

public class HumanoidBuilder : MonoBehaviour
{
    public static GameObject CreateSimpleHumanoid()
    {
        GameObject humanoid = new GameObject("SimpleHumanoid");

        // Torso
        GameObject torso = CreateBodyPart("Torso", new Vector3(0.4f, 0.6f, 0.3f), humanoid.transform);
        torso.transform.localPosition = new Vector3(0, 1.0f, 0);

        // Head
        GameObject head = CreateBodyPart("Head", new Vector3(0.3f, 0.3f, 0.3f), torso.transform);
        head.transform.localPosition = new Vector3(0, 0.45f, 0);
        AddCamera(head);

        // Arms
        CreateArm("LeftArm", torso.transform, new Vector3(-0.25f, 0.2f, 0));
        CreateArm("RightArm", torso.transform, new Vector3(0.25f, 0.2f, 0));

        // Legs
        CreateLeg("LeftLeg", torso.transform, new Vector3(-0.1f, -0.3f, 0));
        CreateLeg("RightLeg", torso.transform, new Vector3(0.1f, -0.3f, 0));

        // Add articulation body for physics
        AddArticulationBody(humanoid);

        return humanoid;
    }

    static GameObject CreateBodyPart(string name, Vector3 size, Transform parent)
    {
        GameObject part = GameObject.CreatePrimitive(PrimitiveType.Cube);
        part.name = name;
        part.transform.SetParent(parent);
        part.transform.localScale = size;
        part.GetComponent<Renderer>().material.color = new Color(0.8f, 0.8f, 0.8f);
        return part;
    }

    static void CreateArm(string name, Transform parent, Vector3 offset)
    {
        GameObject arm = CreateBodyPart(name, new Vector3(0.1f, 0.5f, 0.1f), parent);
        arm.transform.localPosition = offset;
    }

    static void CreateLeg(string name, Transform parent, Vector3 offset)
    {
        GameObject leg = CreateBodyPart(name, new Vector3(0.15f, 0.6f, 0.15f), parent);
        leg.transform.localPosition = offset;
    }

    static void AddCamera(GameObject head)
    {
        GameObject cam = new GameObject("HeadCamera");
        cam.transform.SetParent(head.transform);
        cam.transform.localPosition = new Vector3(0, 0, 0.2f);
        cam.AddComponent<Camera>();
    }

    static void AddArticulationBody(GameObject root)
    {
        ArticulationBody ab = root.AddComponent<ArticulationBody>();
        ab.immovable = false;
        ab.useGravity = true;
    }
}
```

## Communicating with ROS 2

### Publishing Messages from Unity to ROS 2

Create a publisher script:

```csharp
// ROSPublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity/position";
    public float publishFrequency = 10f;

    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1f / publishFrequency)
        {
            Vector3 pos = transform.position;
            StringMsg msg = new StringMsg
            {
                data = $"Position: x={pos.x:F2}, y={pos.y:F2}, z={pos.z:F2}"
            };

            ros.Publish(topicName, msg);
            timeElapsed = 0;
        }
    }
}
```

Attach this script to your robot in Unity.

### Subscribing to ROS 2 Messages in Unity

Create a subscriber script:

```csharp
// ROSSubscriber.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class ROSSubscriber : MonoBehaviour
{
    public string topicName = "cmd_vel";

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(topicName, MoveRobot);
    }

    void MoveRobot(TwistMsg twist)
    {
        // Convert ROS Twist to Unity movement
        float linearX = (float)twist.linear.x;
        float angularZ = (float)twist.angular.z;

        // Apply to robot
        transform.Translate(Vector3.forward * linearX * Time.deltaTime);
        transform.Rotate(Vector3.up, angularZ * Mathf.Rad2Deg * Time.deltaTime);

        Debug.Log($"Received cmd_vel: linear.x={linearX}, angular.z={angularZ}");
    }
}
```

### Publishing Camera Images

Create a camera publisher:

```csharp
// CameraPublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    public Camera robotCamera;
    public string topicName = "camera/image_raw";
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    public float publishFrequency = 10f;

    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private ROSConnection ros;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        renderTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        robotCamera.targetTexture = renderTexture;
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1f / publishFrequency)
        {
            PublishImage();
            timeElapsed = 0;
        }
    }

    void PublishImage()
    {
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, resolutionWidth, resolutionHeight), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time - (int)Time.time) * 1e9)
                },
                frame_id = "camera_link"
            },
            height = (uint)resolutionHeight,
            width = (uint)resolutionWidth,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(resolutionWidth * 3),
            data = texture2D.GetRawTextureData()
        };

        ros.Publish(topicName, msg);
    }
}
```

## Joint Control

### Articulation Body for Realistic Physics

Unity's Articulation Body system provides accurate multi-body dynamics:

```csharp
// JointController.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class JointController : MonoBehaviour
{
    public ArticulationBody joint;
    public string topicName = "joint_command";

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<Float64Msg>(topicName, UpdateJoint);
    }

    void UpdateJoint(Float64Msg msg)
    {
        if (joint != null)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.target = (float)msg.data * Mathf.Rad2Deg;
            joint.xDrive = drive;
        }
    }
}
```

### Configure Articulation Chain

For a humanoid robot, set up the articulation chain:

1. Root (Torso): **ArticulationBody** with **immovable = false**
2. Child joints: **ArticulationBody** with appropriate joint types
3. Configure drives for each joint (stiffness, damping, force limits)

## Creating Environments

### Import Assets

1. **Unity Asset Store**: Search for "warehouse", "office", "outdoor"
2. **Free Assets**: Polygon Prototype Pack, Modular Sci-Fi
3. **Custom Models**: Import from Blender, 3DS Max

### Add Physics Materials

```csharp
// Create in Unity Editor:
// Assets > Create > Physic Material
// - Static Friction: 0.6
// - Dynamic Friction: 0.4
// - Bounciness: 0.0
```

Apply to floor and objects.

## Debugging and Visualization

### ROS Visualizer in Unity

Install the visualizer package:
```text
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations
```

Use it to visualize ROS topics in Unity scene:
- Transform messages (TF)
- Markers
- Point clouds
- Laser scans

### Console Logging

```csharp
Debug.Log($"Joint angle: {jointAngle}");
Debug.LogWarning("Connection lost!");
Debug.LogError("Invalid joint configuration");
```

View logs in **Window > General > Console**.

## Performance Optimization

1. **Reduce Physics Iterations**: Edit > Project Settings > Physics
2. **Optimize Rendering**: Use occlusion culling, LOD groups
3. **Simplify Colliders**: Use primitive colliders when possible
4. **Limit Update Rate**: Don't publish at 60Hz unless needed
5. **Async Operations**: Use coroutines for heavy operations

## Example: Complete Teleoperation

```csharp
// TeleoperationSystem.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class TeleoperationSystem : MonoBehaviour
{
    public ArticulationBody robotBase;
    public Camera robotCamera;
    public float moveSpeed = 2.0f;
    public float rotateSpeed = 90.0f;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("cmd_vel", HandleTwist);
        ros.RegisterPublisher<ImageMsg>("camera/image");
    }

    void HandleTwist(TwistMsg twist)
    {
        float linear = (float)twist.linear.x * moveSpeed;
        float angular = (float)twist.angular.z * rotateSpeed;

        Vector3 force = transform.forward * linear;
        robotBase.AddForce(force);
        robotBase.AddTorque(Vector3.up * angular);
    }
}
```

## Troubleshooting

**Connection Failed:**
- Verify ROS-TCP endpoint is running
- Check IP address and port in ROS Settings
- Ensure firewall allows port 10000

**Robot Falls Through Floor:**
- Add MeshCollider to ground
- Check robot's ArticulationBody mass
- Verify collision layers

**Poor Performance:**
- Lower camera resolution
- Reduce physics update rate
- Simplify collision meshes
- Disable shadows for non-critical objects

## Next Steps

In the next lesson, you'll explore NVIDIA Isaac Sim for even more advanced physics simulation and GPU-accelerated sensor simulation.

## Further Reading

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector Documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity Manual](https://docs.unity3d.com/Manual/index.html)
