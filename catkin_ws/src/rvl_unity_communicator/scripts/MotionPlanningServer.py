#!/usr/bin/env python

import rospy
import sys
import moveit_commander

from math import pi, tau, fabs, dist, degrees, radians

from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface

from rvl_unity_communicator.srv import ArmPlanning, ArmPlanningRequest, ArmPlanningResponse

def handle_arm_motion_planning(request: ArmPlanningRequest, planner: MoveGroupCommander):
    joint_goal = planner.get_current_joint_values()
    joint_goal[0] = radians(request.shoulder_pan_joint)
    joint_goal[1] = radians(request.shoulder_lift_joint)
    joint_goal[2] = radians(request.elbow_joint)
    joint_goal[3] = radians(request.wrist_1_joint)
    joint_goal[4] = radians(request.wrist_2_joint)
    joint_goal[5] = radians(request.wrist_3_joint)
    planner.go(joint_goal, wait=True)
    planner.stop()

def moveit_planning_server():
    rospy.init_node('unity_moveit_planning_server', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    arm = MoveGroupCommander('arm')
    gripper = MoveGroupCommander('gripper')

    arm_srv = rospy.Service('/arm_planning', ArmPlanning, lambda request: handle_arm_motion_planning(request, arm))

    rospy.spin()

if __name__ == '__main__':
    moveit_planning_server()

# ---------------------------------------------------------------------------- #
#                                   CODE DUMP                                  #
# ---------------------------------------------------------------------------- #

# joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)

# js = JointState()
# js.name = ["shoulder_pan_joint",
#             "shoulder_lift_joint",
#             "elbow_joint",
#             "wrist_1_joint",
#             "wrist_2_joint",
#             "wrist_3_joint",
#             "left_distal_joint",
#             "left_proximal_joint",
#             "knuckle_joint",
#             "right_distal_joint",
#             "right_proximal_joint",
#             "right_knuckle_joint"]
# js.position = [0.0 for _ in js.name]
# js.velocity = []
# js.effort   = []
# js.header.stamp = rospy.Time.now()
# js.header.frame_id = ''