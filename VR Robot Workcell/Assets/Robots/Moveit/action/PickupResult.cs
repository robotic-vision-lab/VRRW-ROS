//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Moveit
{
    [Serializable]
    public class PickupResult : Message
    {
        public const string k_RosMessageName = "moveit_msgs/Pickup";
        public override string RosMessageName => k_RosMessageName;

        //  The overall result of the pickup attempt
        public MoveItErrorCodesMsg error_code;
        //  The full starting state of the robot at the start of the trajectory
        public RobotStateMsg trajectory_start;
        //  The trajectory that moved group produced for execution
        public RobotTrajectoryMsg[] trajectory_stages;
        public string[] trajectory_descriptions;
        //  The performed grasp, if attempt was successful
        public GraspMsg grasp;
        //  The amount of time in seconds it took to complete the plan
        public double planning_time;

        public PickupResult()
        {
            this.error_code = new MoveItErrorCodesMsg();
            this.trajectory_start = new RobotStateMsg();
            this.trajectory_stages = new RobotTrajectoryMsg[0];
            this.trajectory_descriptions = new string[0];
            this.grasp = new GraspMsg();
            this.planning_time = 0.0;
        }

        public PickupResult(MoveItErrorCodesMsg error_code, RobotStateMsg trajectory_start, RobotTrajectoryMsg[] trajectory_stages, string[] trajectory_descriptions, GraspMsg grasp, double planning_time)
        {
            this.error_code = error_code;
            this.trajectory_start = trajectory_start;
            this.trajectory_stages = trajectory_stages;
            this.trajectory_descriptions = trajectory_descriptions;
            this.grasp = grasp;
            this.planning_time = planning_time;
        }

        public static PickupResult Deserialize(MessageDeserializer deserializer) => new PickupResult(deserializer);

        private PickupResult(MessageDeserializer deserializer)
        {
            this.error_code = MoveItErrorCodesMsg.Deserialize(deserializer);
            this.trajectory_start = RobotStateMsg.Deserialize(deserializer);
            deserializer.Read(out this.trajectory_stages, RobotTrajectoryMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.trajectory_descriptions, deserializer.ReadLength());
            this.grasp = GraspMsg.Deserialize(deserializer);
            deserializer.Read(out this.planning_time);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.error_code);
            serializer.Write(this.trajectory_start);
            serializer.WriteLength(this.trajectory_stages);
            serializer.Write(this.trajectory_stages);
            serializer.WriteLength(this.trajectory_descriptions);
            serializer.Write(this.trajectory_descriptions);
            serializer.Write(this.grasp);
            serializer.Write(this.planning_time);
        }

        public override string ToString()
        {
            return "PickupResult: " +
            "\nerror_code: " + error_code.ToString() +
            "\ntrajectory_start: " + trajectory_start.ToString() +
            "\ntrajectory_stages: " + System.String.Join(", ", trajectory_stages.ToList()) +
            "\ntrajectory_descriptions: " + System.String.Join(", ", trajectory_descriptions.ToList()) +
            "\ngrasp: " + grasp.ToString() +
            "\nplanning_time: " + planning_time.ToString();
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
