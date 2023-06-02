using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.MycobotCommunication;
using RosMessageTypes.UnityRoboticsDemo;
using RosMessageTypes.Moveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class MycobotTrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;
    const float k_RobotMoveRadians = Mathf.PI/8;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "mycobot_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }
    string m_RosMoveToPoseServiceName = "niryo_movetopose";
    public string RosMoveToPoseServiceName { get => m_RosMoveToPoseServiceName; set => m_RosMoveToPoseServiceName = value; }

    [SerializeField]
    GameObject m_Mycobot;
    public GameObject Mycobot { get => m_Mycobot; set => m_Mycobot = value; }
    [SerializeField]
    GameObject m_Gripper;
    public GameObject Gripper { get => m_Gripper; set => m_Gripper = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = new Vector3(0, 1.1f, 0.075f) * 0.1f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    // Animator
    Animator m_GripperAnimator;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);
        m_Ros.RegisterRosService<MoveToPoseServiceRequest, MoveToPoseServiceResponse>(m_RosMoveToPoseServiceName);

        m_Ros.ImplementService<ObjectPoseServiceRequest, ObjectPoseServiceResponse>("unity/object_pose_svc", GetObjectPose);
        m_Ros.ImplementService<GetSceneObjectsServiceRequest, GetSceneObjectsServiceResponse>("unity/get_scene_objects_svc", GetSceneObjects);

        m_Ros.Subscribe<PickAndPlaceMsg>("unity/pick_and_place", StartPickAndPlace);

        m_Ros.Subscribe<MoveCommandMsg>("unity/move_commands", OnMoveCommand);

        m_Ros.Subscribe<RosMessageTypes.Geometry.PoseMsg>("unity/move_to_pose", OnMoveToPose);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += MycobotSourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_Mycobot.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        m_GripperAnimator = Gripper.GetComponent<Animator>();
    }

    void StartPickAndPlace(PickAndPlaceMsg request)
    {
        // process the service request
        Debug.Log($"Received Pick And Place request");

        PickAndPlace(request.pick_pose, request.place_pose);
    }

    void OnMoveCommand(MoveCommandMsg request)
    {
        // process the service request
        Debug.Log($"Received move command");

        switch (request.move_command)
        {
            case "rotate_left":
                StartCoroutine(RotateArm(RotateDirection.LEFT, request.radians));
                break;

            case "rotate_right":
                StartCoroutine(RotateArm(RotateDirection.RIGHT, request.radians));
                break;

            case "tilt_up":
                StartCoroutine(RotateArm(RotateDirection.UP, request.radians));
                break;

            case "tilt_down":
                StartCoroutine(RotateArm(RotateDirection.DOWN, request.radians));
                break;

            case "move_forward":
                StartCoroutine(RotateArm(RotateDirection.FORWARD, request.radians));
                break;
            
            case "move_backward":
                StartCoroutine(RotateArm(RotateDirection.BACKWARD, request.radians));
                break;
            
            case "grasp":
                CloseGripper();
                break;

            case "release":
                OpenGripper();
                break;

            default:
                Debug.LogError($"Received invalid move command");
                break;
        }
    }

    public enum RotateDirection
    {
        LEFT,
        RIGHT,
        UP,
        DOWN,
        FORWARD,
        BACKWARD
    }

    public void RotateArmLeft()
    {
        StartCoroutine(RotateArm(RotateDirection.LEFT, k_RobotMoveRadians));
    }

    public void RotateArmRight()
    {
        StartCoroutine(RotateArm(RotateDirection.RIGHT, k_RobotMoveRadians));
    }

    public void TiltArmUp()
    {
        StartCoroutine(RotateArm(RotateDirection.UP, k_RobotMoveRadians));
    }

    public void TiltArmDown()
    {
        StartCoroutine(RotateArm(RotateDirection.DOWN, k_RobotMoveRadians));
    }

    public void ExtendArmForward()
    {
        StartCoroutine(RotateArm(RotateDirection.FORWARD, k_RobotMoveRadians));
    }

    public void ExtendArmBackward()
    {
        StartCoroutine(RotateArm(RotateDirection.BACKWARD, k_RobotMoveRadians));
    }

    public IEnumerator RotateArm(RotateDirection direction, double radians)
    {
        Debug.Log($"Rotating arm {direction.ToString()} by {radians} radians.");

        float degrees = (float)radians * Mathf.Rad2Deg;

        int joint = 0;
        float targetOffset = 0;
        switch (direction)
        {
            case RotateDirection.LEFT:
                joint = 0;
                targetOffset = degrees;
                break;

            case RotateDirection.RIGHT:
                joint = 0;
                targetOffset = -1 * degrees;
                break;

            case RotateDirection.UP:
                joint = 1;
                targetOffset = degrees;
                break;

            case RotateDirection.DOWN:
                joint = 1;
                targetOffset = -1 * degrees;
                break;

            case RotateDirection.FORWARD:
                joint = 2;
                targetOffset = degrees;
                break;

            case RotateDirection.BACKWARD:
                joint = 2;
                targetOffset = -1 * degrees;
                break;

            default:
                break;
        }

        for (int i=0; i < Mathf.Abs(targetOffset); i++)
        {
            var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
            joint1XDrive.target += (targetOffset > 0) ? 1 : -1;
            //joint1XDrive.targetVelocity = 0.000001f;
            m_JointArticulationBodies[joint].xDrive = joint1XDrive;

            // Wait for robot to achieve pose for all joint assignments
            yield return new WaitForSeconds(k_JointAssignmentWait); 
        }

        // Wait for robot to achieve pose for all joint assignments
        //yield return new WaitForSeconds(k_JointAssignmentWait);
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        m_GripperAnimator.SetTrigger("TrClose");
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        m_GripperAnimator.SetTrigger("TrOpen");
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>NiryoMoveitJoints</returns>
    MycobotAnglesMsg CurrentJointConfig()
    {
        var joints = new MycobotAnglesMsg();

        joints.joint_1 = m_JointArticulationBodies[0].jointPosition[0];
        joints.joint_2 = m_JointArticulationBodies[1].jointPosition[0];
        joints.joint_3 = m_JointArticulationBodies[2].jointPosition[0];
        joints.joint_4 = m_JointArticulationBodies[3].jointPosition[0];
        joints.joint_5 = m_JointArticulationBodies[4].jointPosition[0];
        joints.joint_6 = m_JointArticulationBodies[5].jointPosition[0];

        return joints;
    }

    private string[] scene_objects = {
        "blue_block",
        "yellow_block",
        "green_block",
        "red_bin",
        "black_bin",
        "orange_bin"
    };

    private GetSceneObjectsServiceResponse GetSceneObjects(GetSceneObjectsServiceRequest request)
    {
        // process the service request
        Debug.Log("Received request to get scene objects");

        List<SceneObjectMsg> found_objects = new List<SceneObjectMsg>();
        
        // Find a game object with the requested name
        foreach (string object_name in scene_objects)
        {
            GameObject gameObject = GameObject.Find(object_name);
            if (gameObject)
            {
                SceneObjectMsg sceneObj = new SceneObjectMsg();

                string[] tokens = object_name.Split('_');
                sceneObj.color = tokens[0];
                sceneObj.object_type = tokens[1];

                // Fill-in the response with the object pose converted from Unity coordinate to ROS coordinate system
                sceneObj.pose.position = (gameObject.transform.position + m_PickPoseOffset).To<FLU>();
                sceneObj.pose.orientation = Quaternion.Euler(0, gameObject.transform.eulerAngles.y, -90).To<FLU>();

                found_objects.Add(sceneObj);
            }
        }

        GetSceneObjectsServiceResponse response = new GetSceneObjectsServiceResponse();
        response.scene_objects = found_objects.ToArray();

        return response;
    }

    private ObjectPoseServiceResponse GetObjectPose(ObjectPoseServiceRequest request)
    {
        // process the service request
        Debug.Log("Received request for object: " + request.object_name);

        // prepare a response
        ObjectPoseServiceResponse objectPoseResponse = new ObjectPoseServiceResponse();
        // Find a game object with the requested name
        GameObject gameObject = GameObject.Find(request.object_name);
        if (gameObject)
        {
            // Fill-in the response with the object pose converted from Unity coordinate to ROS coordinate system
            objectPoseResponse.object_pose.position = (gameObject.transform.position + m_PickPoseOffset).To<FLU>();
            objectPoseResponse.object_pose.orientation = Quaternion.Euler(0, gameObject.transform.eulerAngles.y, 0).To<FLU>();
        }

        return objectPoseResponse;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(0, 90, -90).To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(0, 90, -90).To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PickAndPlace(PoseMsg pick_pose, PoseMsg place_pose)
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = pick_pose;

        // Place Pose
        request.place_pose = place_pose;

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, OnMoverServiceResponse);
    }

    void OnMoverServiceResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response.trajectories));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Create a new MoveToPoseServiceRequest with the current values of the robot's joint angles and
    ///     the target position and rotation.
    ///     Call the MoveToPoseService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void OnMoveToPose(PoseMsg target_pose)
    {
        Debug.Log("Received request to Move To Pose");

        var request = new MoveToPoseServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Target Pose
        request.target_pose = target_pose;

        m_Ros.SendServiceMessage<MoveToPoseServiceResponse>(m_RosMoveToPoseServiceName, request, OnMoveToPoseServiceResponse);
    }

    void OnMoveToPoseServiceResponse(MoveToPoseServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response.trajectories));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoveToPoseService.");
        }
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(RobotTrajectoryMsg[] trajectories)
    {
        if (trajectories != null && trajectories.Length > 0)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}
