//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityMoveitPlanning
{
    [Serializable]
    public class ForwardKinematicsResponse : Message
    {
        public const string k_RosMessageName = "unity_moveit_planning/ForwardKinematics";
        public override string RosMessageName => k_RosMessageName;

        public Moveit.RobotTrajectoryMsg[] trajectories;

        public ForwardKinematicsResponse()
        {
            this.trajectories = new Moveit.RobotTrajectoryMsg[0];
        }

        public ForwardKinematicsResponse(Moveit.RobotTrajectoryMsg[] trajectories)
        {
            this.trajectories = trajectories;
        }

        public static ForwardKinematicsResponse Deserialize(MessageDeserializer deserializer) => new ForwardKinematicsResponse(deserializer);

        private ForwardKinematicsResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.trajectories, Moveit.RobotTrajectoryMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.trajectories);
            serializer.Write(this.trajectories);
        }

        public override string ToString()
        {
            return "ForwardKinematicsResponse: " +
            "\ntrajectories: " + System.String.Join(", ", trajectories.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
