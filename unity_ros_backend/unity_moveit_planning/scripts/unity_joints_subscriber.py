#!/usr/bin/python3

import rospy

from unity_moveit_planning.msg import URJoints

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard:\n%s", data)

def listener():
    rospy.init_node('Unity_Current_Joint_Subscriber', anonymous=True)
    rospy.Subscriber("unity_current_joints", URJoints, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
