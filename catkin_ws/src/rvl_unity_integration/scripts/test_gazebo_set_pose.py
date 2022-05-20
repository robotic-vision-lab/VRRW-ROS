#!/usr/bin/env python3

import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose

def update_gazebo_pose(pose_msg:Pose):
    state_msg = ModelState()
    state_msg.model_name = 'hmd_camera'
    state_msg.pose.position.x = pose_msg.position.x
    state_msg.pose.position.y = pose_msg.position.y
    state_msg.pose.position.z = pose_msg.position.z
    state_msg.pose.orientation.x = pose_msg.orientation.x
    state_msg.pose.orientation.y = pose_msg.orientation.y
    state_msg.pose.orientation.z = pose_msg.orientation.z
    state_msg.pose.orientation.w = pose_msg.orientation.w

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
        del resp
        del state_msg
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def main():
    rospy.init_node('hmd_pose_updater_node', anonymous=True)
    print('[RVL Unity] Waiting for topics from Unity...')
    rospy.wait_for_message('/unity/hmd_pose', Pose)
    rospy.wait_for_service('/gazebo/set_model_state')
    print('[RVL Unity] Pose updating...')
    sub = rospy.Subscriber('/unity/hmd_pose', Pose, callback=update_gazebo_pose)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

