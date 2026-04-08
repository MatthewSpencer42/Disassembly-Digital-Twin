using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Management;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using TMPro;

/// <summary>
/// OpenXR Hand Tracking Teleoperation Publisher for XArm5
/// Publishes hand keypoints and gestures via ROS TCP Connector.
/// Follows the same data format as the OVR version for compatibility with xarm5_teleop_node.py
/// </summary>
public class OpenXRGestureDetectorROS : MonoBehaviour
{
    [Header("ROS Configuration")]
    [SerializeField] private string rosIP = "192.168.1.100";
    [SerializeField] private int rosPort = 10000;

    [Header("ROS Topics")]
    [SerializeField] private string handKeypointsTopic = "/xarm5/hand_keypoints";
    [SerializeField] private string handPoseTopic = "/teleop_hand_tracking/right/wrist";
    [SerializeField] private string gripperCommandTopic = "/teleop_hand_tracking/right/pinky_pinch";
    [SerializeField] private string teleopStateTopic = "/teleop_hand_tracking/right/fist";
    [SerializeField] private string robotStateTopic = "/teleop_status/right_arm_enabled";
    

    [Header("Hand Selection")]
    [SerializeField] private Handedness handedness = Handedness.Right;

    [Header("Controller Settings")]
    [SerializeField] private float publishRate = 90.0f;     // Hz (matches Open-Teach)
    [SerializeField] private float triggerThreshold = 0.5f;  // Trigger press threshold (0-1)

    [Header("UI Display")]
    [SerializeField] private TextMeshProUGUI teleopStateText;
    [SerializeField] private TextMeshProUGUI robotStateText;
    [SerializeField] private TextMeshProUGUI gripperStateText;
    [SerializeField] private Color activeColor = Color.green;
    [SerializeField] private Color pausedColor = Color.yellow;
    [SerializeField] private Color disconnectedColor = Color.red;

    [Header("Manual Override")]
    [SerializeField] private bool useTeleopOverride = false;
    [SerializeField] private bool teleopOverrideValue = false;

    [Header("Debug")]
    [SerializeField] private bool enableLogging = true;
    [SerializeField] private float logInterval = 1.0f;

    // ROS Connection
    private ROSConnection ros;
    private static bool publishersRegistered = false;

    // OpenXR Hand Subsystem
    private XRHandSubsystem xrHandSubsystem;

    // State
    private bool isTeleoperating = false;
    private bool gripperOpen = true;
    private bool wasLeftIndexPressed = false;
    private bool wasRightIndexPressed = false;

    // Controller devices
    private InputDevice leftController;
    private InputDevice rightController;
    private bool controllersFound = false;

    // Timing
    private float lastPublishTime = 0f;
    private float lastLogTime = 0f;

    // Robot state feedback
    private string currentRobotState = "Disconnected";

    // Joint mapping from OpenXR to our array format (matching OVRSkeleton order)
    // OpenXR XRHandJointID: Wrist=0, Palm=1, ThumbMetacarpal=2, ThumbProximal=3, etc.
    private static readonly XRHandJointID[] jointMapping = new XRHandJointID[]
    {
        XRHandJointID.Wrist,              // 0 - Wrist
        XRHandJointID.Palm,               // 1 - Palm (ForearmStub equivalent)
        XRHandJointID.ThumbMetacarpal,    // 2 - Thumb0
        XRHandJointID.ThumbProximal,      // 3 - Thumb1
        XRHandJointID.ThumbDistal,        // 4 - Thumb2
        XRHandJointID.ThumbTip,           // 5 - Thumb3 (using tip as placeholder)
        XRHandJointID.IndexProximal,      // 6 - Index1 (knuckle) - IMPORTANT for hand frame
        XRHandJointID.IndexIntermediate,  // 7 - Index2
        XRHandJointID.IndexDistal,        // 8 - Index3
        XRHandJointID.MiddleProximal,     // 9 - Middle1
        XRHandJointID.MiddleIntermediate, // 10 - Middle2
        XRHandJointID.MiddleDistal,       // 11 - Middle3
        XRHandJointID.RingProximal,       // 12 - Ring1
        XRHandJointID.RingIntermediate,   // 13 - Ring2
        XRHandJointID.RingDistal,         // 14 - Ring3
        XRHandJointID.LittleMetacarpal,   // 15 - Pinky0
        XRHandJointID.LittleProximal,     // 16 - Pinky1 (knuckle) - IMPORTANT for hand frame
        XRHandJointID.LittleIntermediate, // 17 - Pinky2
        XRHandJointID.LittleDistal,       // 18 - Pinky3
        XRHandJointID.ThumbTip,           // 19 - ThumbTip
        XRHandJointID.IndexTip,           // 20 - IndexTip
        XRHandJointID.MiddleTip,          // 21 - MiddleTip
        XRHandJointID.RingTip,            // 22 - RingTip
        XRHandJointID.LittleTip,          // 23 - PinkyTip
    };

    void Start()
    {
        // Initialize ROS connection
        ROSConnection.GetOrCreateInstance().ConnectOnStart = true;
        ros = ROSConnection.GetOrCreateInstance();

        // Register publishers (only once to avoid warnings)
        if (!publishersRegistered)
        {
            ros.RegisterPublisher<Float32MultiArrayMsg>(handKeypointsTopic);
            ros.RegisterPublisher<PoseStampedMsg>(handPoseTopic);
            ros.RegisterPublisher<BoolMsg>(gripperCommandTopic);
            ros.RegisterPublisher<BoolMsg>(teleopStateTopic);
            ros.Subscribe<BoolMsg>(robotStateTopic, OnRobotStateReceived);
            publishersRegistered = true;
        }

        // Initialize OpenXR Hand Subsystem
        InitializeHandSubsystem();

        Debug.Log($"[OpenXRGestureDetectorROS] Initialized - Publishing to {rosIP}:{rosPort}");
        Debug.Log($"[OpenXRGestureDetectorROS] Using {handedness} hand");
    }

    void InitializeHandSubsystem()
    {
        if (XRGeneralSettings.Instance != null &&
            XRGeneralSettings.Instance.Manager != null &&
            XRGeneralSettings.Instance.Manager.activeLoader != null)
        {
            xrHandSubsystem = XRGeneralSettings.Instance.Manager.activeLoader.GetLoadedSubsystem<XRHandSubsystem>();
        }

        if (xrHandSubsystem == null)
        {
            Debug.LogError("[OpenXRGestureDetectorROS] XR Hand Subsystem not found! Ensure XR Hands package is installed and enabled.");
        }
        else
        {
            Debug.Log("[OpenXRGestureDetectorROS] XR Hand Subsystem initialized successfully");
        }
    }

    void Update()
    {
        // Try to initialize subsystem if not available
        if (xrHandSubsystem == null)
        {
            InitializeHandSubsystem();
            return;
        }

        // Try to find controllers if not found
        if (!controllersFound)
        {
            FindControllers();
        }

        // Rate limiting
        if (Time.time - lastPublishTime < (1.0f / publishRate))
            return;

        // Get the selected hand
        XRHand hand = (handedness == Handedness.Left) ? xrHandSubsystem.leftHand : xrHandSubsystem.rightHand;

        if (!hand.isTracked)
            return;

        // Process controller inputs
        ProcessControllerInput();

        // Publish hand data only when teleoperating
        if (isTeleoperating)
        {
            PublishHandKeypoints(hand);
            PublishHandPose(hand);
        }

        // Update UI
        UpdateUI();

        lastPublishTime = Time.time;
    }

    void FindControllers()
    {
        var leftDevices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.LeftHand, leftDevices);
        if (leftDevices.Count > 0)
            leftController = leftDevices[0];

        var rightDevices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, rightDevices);
        if (rightDevices.Count > 0)
            rightController = rightDevices[0];

        controllersFound = leftController.isValid && rightController.isValid;
        if (controllersFound)
            Debug.Log("[OpenXRGestureDetectorROS] Controllers found");
    }

    void ProcessControllerInput()
    {
        // Handle manual teleop override
        if (useTeleopOverride)
        {
            if (isTeleoperating != teleopOverrideValue)
            {
                isTeleoperating = teleopOverrideValue;
                PublishTeleopState();
                Debug.Log($"[OpenXRGestureDetectorROS] Teleop Override: {(isTeleoperating ? "STARTED" : "PAUSED")}");
            }
            return;
        }

        if (!controllersFound)
            return;

        // Dead man's switch: left hand grip trigger must be held
        bool deadManActive = false;
        if (leftController.TryGetFeatureValue(CommonUsages.gripButton, out bool gripPressed))
        {
            deadManActive = gripPressed;
        }
        else if (leftController.TryGetFeatureValue(CommonUsages.grip, out float gripValue))
        {
            deadManActive = gripValue > triggerThreshold;
        }

        // If dead man's switch is released, pause teleop
        if (!deadManActive && isTeleoperating)
        {
            isTeleoperating = false;
            PublishTeleopState();
            Debug.Log("[OpenXRGestureDetectorROS] Teleop PAUSED (dead man's switch released)");
        }

        // Left index trigger (front trigger) - Toggle teleop on/off
        bool leftIndexPressed = false;
        if (leftController.TryGetFeatureValue(CommonUsages.triggerButton, out bool leftTriggerButton))
        {
            leftIndexPressed = leftTriggerButton;
        }
        else if (leftController.TryGetFeatureValue(CommonUsages.trigger, out float leftTriggerValue))
        {
            leftIndexPressed = leftTriggerValue > triggerThreshold;
        }

        if (leftIndexPressed && !wasLeftIndexPressed && deadManActive)
        {
            isTeleoperating = !isTeleoperating;
            PublishTeleopState();
            Debug.Log($"[OpenXRGestureDetectorROS] Teleop: {(isTeleoperating ? "STARTED" : "PAUSED")}");
        }
        wasLeftIndexPressed = leftIndexPressed;

        // Right index trigger (front trigger) - Toggle gripper
        bool rightIndexPressed = false;
        if (rightController.TryGetFeatureValue(CommonUsages.triggerButton, out bool rightTriggerButton))
        {
            rightIndexPressed = rightTriggerButton;
        }
        else if (rightController.TryGetFeatureValue(CommonUsages.trigger, out float rightTriggerValue))
        {
            rightIndexPressed = rightTriggerValue > triggerThreshold;
        }

        if (rightIndexPressed && !wasRightIndexPressed)
        {
            gripperOpen = !gripperOpen;
            PublishGripperPulse();
            Debug.Log($"[OpenXRGestureDetectorROS] Gripper: {(gripperOpen ? "OPEN" : "CLOSED")}");
        }
        wasRightIndexPressed = rightIndexPressed;
    }

    void PublishHandKeypoints(XRHand hand)
    {
        Float32MultiArrayMsg keypointsMsg = new Float32MultiArrayMsg();
        List<float> data = new List<float>();

        // Add teleop state as first value (1.0 = active, 0.0 = paused)
        data.Add(isTeleoperating ? 1.0f : 0.0f);

        // Add gripper state as second value (1.0 = open, 0.0 = closed)
        data.Add(gripperOpen ? 1.0f : 0.0f);

        // Add all joint positions (24 joints x 3 coordinates = 72 values)
        foreach (var jointId in jointMapping)
        {
            if (hand.GetJoint(jointId).TryGetPose(out Pose jointPose))
            {
                // Convert Unity coordinates to ROS coordinates
                // Unity: X-right, Y-up, Z-forward
                // ROS: X-forward, Y-left, Z-up
                data.Add(jointPose.position.z);   // Unity Z -> ROS X
                data.Add(-jointPose.position.x);  // Unity X -> ROS -Y
                data.Add(jointPose.position.y);   // Unity Y -> ROS Z
            }
            else
            {
                // Joint not tracked, add zeros
                data.Add(0.0f);
                data.Add(0.0f);
                data.Add(0.0f);
            }
        }

        keypointsMsg.data = data.ToArray();

        // Set layout info
        keypointsMsg.layout = new MultiArrayLayoutMsg();
        keypointsMsg.layout.dim = new MultiArrayDimensionMsg[]
        {
            new MultiArrayDimensionMsg
            {
                label = "keypoints",
                size = (uint)jointMapping.Length,
                stride = (uint)data.Count
            }
        };

        ros.Publish(handKeypointsTopic, keypointsMsg);

        // Debug logging
        if (enableLogging && Time.time - lastLogTime > logInterval)
        {
            if (hand.GetJoint(XRHandJointID.Wrist).TryGetPose(out Pose wristPose))
            {
                Debug.Log($"[OpenXRGestureDetectorROS] Wrist: ({wristPose.position.z:F3}, {-wristPose.position.x:F3}, {wristPose.position.y:F3}) " +
                          $"Teleop: {isTeleoperating} Gripper: {(gripperOpen ? "Open" : "Closed")}");
            }
            lastLogTime = Time.time;
        }
    }

    void PublishHandPose(XRHand hand)
    {
        if (!hand.GetJoint(XRHandJointID.Wrist).TryGetPose(out Pose wristPose))
            return;

        PoseStampedMsg poseMsg = new PoseStampedMsg();

        // Header
        poseMsg.header = new HeaderMsg
        {
            frame_id = "world",
            stamp = GetROSTimeStamp()
        };

        // Position (Unity -> ROS coordinate conversion)
        poseMsg.pose.position = new PointMsg
        {
            x = wristPose.position.z,
            y = -wristPose.position.x,
            z = wristPose.position.y
        };

        // Orientation (Unity -> ROS coordinate conversion)
        poseMsg.pose.orientation = new QuaternionMsg
        {
            x = wristPose.rotation.z,
            y = -wristPose.rotation.x,
            z = wristPose.rotation.y,
            w = wristPose.rotation.w
        };

        ros.Publish(handPoseTopic, poseMsg);
    }

    void PublishGripperPulse()
    {
        ros.Publish(gripperCommandTopic, new BoolMsg { data = true });
        ros.Publish(gripperCommandTopic, new BoolMsg { data = false });
    }

    void PublishTeleopState()
    {
        ros.Publish(teleopStateTopic, new BoolMsg { data = true });
        ros.Publish(teleopStateTopic, new BoolMsg { data = false });
    }

    void OnRobotStateReceived(BoolMsg msg)
    {
        currentRobotState = msg.data ? "Enabled" : "Disabled";
        Debug.Log($"[OpenXRGestureDetectorROS] Robot State: {currentRobotState}");
    }

    TimeMsg GetROSTimeStamp()
    {
        double timeInSeconds = Time.timeAsDouble;
        return new TimeMsg
        {
            sec = (int)timeInSeconds,
            nanosec = (uint)((timeInSeconds - (int)timeInSeconds) * 1e9)
        };
    }

    void OnDisable()
    {
        // Stop teleoperation when disabled
        if (isTeleoperating)
        {
            isTeleoperating = false;
            PublishTeleopState();
        }
    }

    void UpdateUI()
    {
        // Update teleop state text
        if (teleopStateText != null)
        {
            if (isTeleoperating)
            {
                teleopStateText.text = "TELEOP: ACTIVE";
                teleopStateText.color = activeColor;
            }
            else
            {
                teleopStateText.text = "TELEOP: PAUSED";
                teleopStateText.color = pausedColor;
            }
        }

        // Update robot state text
        if (robotStateText != null)
        {
            robotStateText.text = $"Robot: {currentRobotState}";
            robotStateText.color = currentRobotState == "Enabled" ? activeColor : pausedColor;
        }

        // Update gripper state text
        if (gripperStateText != null)
        {
            gripperStateText.text = $"Gripper: {(gripperOpen ? "OPEN" : "CLOSED")}";
            gripperStateText.color = gripperOpen ? activeColor : pausedColor;
        }
    }
}
