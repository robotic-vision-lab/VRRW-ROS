#!/usr/bin/python3

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from unity_moveit_planning.msg import URJoints
from unity_moveit_planning.srv import ForwardKinematics

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
        'unity_current_joints': RosPublisher('unity_current_joints', URJoints, queue_size=10),
        'arm_fk_service': RosService('arm_fk_service', ForwardKinematics)
    })
    rospy.spin()

if __name__ == "__main__":
    main()
