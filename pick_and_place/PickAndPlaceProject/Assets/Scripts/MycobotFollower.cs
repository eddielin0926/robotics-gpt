using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.MycobotCommunication;
using RosMessageTypes.UnityRoboticsDemo;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class MycobotFollower : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosTopicName = "/joint_states";
    public string RosTopicName { get => m_RosTopicName; set => m_RosTopicName = value; }

    [SerializeField]
    GameObject m_Mycobot;
    public GameObject Mycobot { get => m_Mycobot; set => m_Mycobot = value; }
    [SerializeField]
    GameObject m_Gripper;
    public GameObject Gripper { get => m_Gripper; set => m_Gripper = value; }

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
        m_Ros.Subscribe<JointStateMsg>(m_RosTopicName, ExecuteJointStates);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += MycobotSourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_Mycobot.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        m_GripperAnimator = Gripper.GetComponent<Animator>();
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

    void ExecuteJointStates(JointStateMsg jointState)
    {
        var jointPositions = jointState.position;
        var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

        // Set the joint values for every joint
        for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
        {
            var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
            joint1XDrive.target = result[joint];
            m_JointArticulationBodies[joint].xDrive = joint1XDrive;
        }
    }
}
