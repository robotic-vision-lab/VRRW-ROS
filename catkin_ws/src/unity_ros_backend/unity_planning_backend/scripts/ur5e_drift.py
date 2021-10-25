#!/usr/bin/python3

import argparse
import sys
import numpy as np
from copy import copy, deepcopy

import rospy
import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from sensor_msgs.msg import JointState

paths = ['./csv/contrl-000-pos-vel.csv',
         './csv/traj-000-gen-pos-vel.csv',
         './csv/traj-001-gen-pos-vel.csv',]

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 
               'elbow_joint', 'wrist_1_joint', 
               'wrist_2_joint', 'wrist_3_joint']


def main():
    print("Initializing node... ")
    rospy.init_node("promp_joint_trajectory_client")

    current_state = rospy.wait_for_message('/joint_states', JointState, 2.0)
    joint_order = [JOINT_NAMES.index(name) for name in current_state.name]

    init_pos = [current_state.position[i] for i in joint_order]
    init_vel = [0, 0, 0, 0, 0, 0]

    print(init_pos)
    
    traj = Trajectory()
    rospy.on_shutdown(traj.stop)
    
    # ---------------------------------------------------------------------------- #
    data_f = paths[1]
    # ---------------------------------------------------------------------------- #
    
    joint_data = np.loadtxt(data_f, delimiter=',')
    print(joint_data)
    print(joint_data.shape)
   
    print("Adding points...")
    traj.add_point(init_pos, init_vel, 1.0)
    time_wait_start = 2.0 # seconds

    input("Press Enter to continue...")

    for i in range(len(joint_data)):
        #if i % 10 == 0:
        #print(joint_data[i, 0], joint_data[i, 1:7], joint_data[i, 7:])
        traj.add_point(joint_data[i,1:7], joint_data[i, 7:], joint_data[i, 0] + time_wait_start)

    input("Press Enter to continue...")

    print("Starting...")
    traj.start()

    print("Waiting...")
    traj.wait(5.0)

    print("Exiting...")
    exit()



# Adapted frin ActionClient example
class Trajectory(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/pos_joint_traj_controller/follow_joint_trajectory/",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.05)
        self._goal.trajectory.joint_names = deepcopy(JOINT_NAMES)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            print(server_up)
            rospy.logerr("Timed out waiting for Joint Trajectory")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        #self.clear(limb)

    def add_point(self, positions, velocities, time):
        point = JointTrajectoryPoint()
        point.positions = deepcopy(positions)
        point.velocities = deepcopy(velocities)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = copy(JOINT_NAMES) 


if __name__ == "__main__":
    main()