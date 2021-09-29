#!/usr/bin/python3

from moveit_commander.move_group import MoveGroupCommander
import rospy
import sys
import math

import moveit_commander as mc
from moveit_commander.conversions import list_to_pose

from unity_moveit_planning.srv import ArmPlanning, ArmPlanningRequest, ArmPlanningResponse
from unity_moveit_planning.msg import URJoints
from sensor_msgs.msg import JointState

def plan_arm_motion(req: ArmPlanningRequest, group: MoveGroupCommander):
    plan = ArmPlanningResponse()

    group.set_joint_value_target([
        req.target.shoulder_pan,
        req.target.shoulder_lift,
        req.target.elbow,
        req.target.wrist_1,
        req.target.wrist_2,
        req.target.wrist_3,
    ])
    
    plan.trajectories.append(group.plan()[1])    

    return plan

if __name__ == "__main__":
    mc.roscpp_initialize(sys.argv)
    rospy.init_node('ros_moveit_server')
    
    robot_commander = mc.RobotCommander()
    planning_scene  = mc.PlanningSceneInterface()
    arm_planning_group  = mc.MoveGroupCommander('arm')
    gripper_planning_group = mc.MoveGroupCommander('gripper')

    s = rospy.Service('arm_planning_service', ArmPlanning, lambda handle: plan_arm_motion(handle, arm_planning_group))
    print("Ready to plan")
    rospy.spin()
    
