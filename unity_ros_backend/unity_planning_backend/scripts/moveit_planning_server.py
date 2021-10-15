#!/usr/bin/python3

from moveit_commander.move_group import MoveGroupCommander

import rospy
import sys
import math

import moveit_commander as mc

from unity_moveit_planning.srv import ForwardKinematics, ForwardKinematicsRequest, ForwardKinematicsResponse, \
                                      UnitySynchronize, UnitySynchronizeRequest, UnitySynchronizeResponse
                                      
from unity_moveit_planning.msg import URJoints

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def serverLog(msg):
    print(f"[Moveit Planning Server] {msg}")

def planCompat(plan):
    return plan[1]

def plan_arm_motion(request: ForwardKinematicsRequest, group: MoveGroupCommander):
    plan = ForwardKinematicsResponse()
    
    group.set_joint_value_target([
        request.target.joint_00,
        request.target.joint_01,
        request.target.joint_02,
        request.target.joint_03,
        request.target.joint_04,
        request.target.joint_05
    ])
    
    moveit_plan = group.plan()
    plan.trajectories.append(planCompat(moveit_plan))
    group.go(wait=False)
    print(moveit_plan)
    group.clear_pose_targets()

    return plan

def sync_arm_position(request: UnitySynchronizeRequest, group: MoveGroupCommander):
    response = UnitySynchronizeResponse()
    response.moveit_current = URJoints()
    curr = group.get_current_joint_values()
    response.moveit_current.joint_00 = curr[0]
    response.moveit_current.joint_01 = curr[1]
    response.moveit_current.joint_02 = curr[2]
    response.moveit_current.joint_03 = curr[3]
    response.moveit_current.joint_04 = curr[4]
    response.moveit_current.joint_05 = curr[5]
    serverLog("Unity arm synchronization service requested!")
    return response

if __name__ == "__main__":
    mc.roscpp_initialize(sys.argv)
    rospy.init_node('ros_moveit_server')
    
    robot_commander = mc.RobotCommander()
    planning_scene  = mc.PlanningSceneInterface()
    
    arm_planning_group = None

    # strict typing?
    arm_planning_group: MoveGroupCommander = mc.MoveGroupCommander('manipulator')
    arm_planning_group.allow_looking(True);
    arm_planning_group.allow_replanning(True);
    arm_planning_group.set_max_acceleration_scaling_factor(0.1);
    arm_planning_group.set_max_velocity_scaling_factor(0.1);
    arm_planning_group.set_num_planning_attempts(10);
    arm_planning_group.set_planning_time(10);
    
    fk_serv = rospy.Service('arm_fk_service', ForwardKinematics, lambda handle: plan_arm_motion(handle, arm_planning_group))
    sync_serv = rospy.Service('backend_sync_service', UnitySynchronize, lambda handle: sync_arm_position(handle, arm_planning_group))
    
    serverLog("MoveIt! ready to plan")
    
    rospy.spin()
    
