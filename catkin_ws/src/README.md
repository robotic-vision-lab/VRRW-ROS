# ROS to Unity Catkin Workspace

## Primary Packages

`ros_tcp_endpoint` contains the [ROS TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) ROS package created by the Unity Robotics Project. It is necessary to register
publishers, subscribers, actions, and services generated in Unity to communicate with ROS.

`ur_ros1_driver` is the [Official UR ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver). It depends on `ur_ros1_description` to function properly.

`unity_ros_backend` provides robot communication interface and control protocols. It also includes additional robots URDFs for UR with grippers (e.g., Robotiq 2F-85) attached.

## Secondary Packages

`moveit_msgs` is necessary for Unity to generate C# equivalent of the messages and services in order to call MoveIt! backend correctly. Once their C# counterparts are generated, this can be discarded.
