#!/usr/bin/env python3

import argparse
import sys
import numpy as np

from copy import copy

import rospy

import rospkg

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# list all packages, equivalent to rospack list
rospack.list() 

# get the file path for rospy_tutorials
datapath = rospack.get_path('rvl_gazebo') + '/scripts/'

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from sensor_msgs.msg import JointState

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 
               'elbow_joint', 'wrist_1_joint', 
               'wrist_2_joint', 'wrist_3_joint']


def main():
    print("Initializing node... ")
    rospy.init_node("promp_joint_trajectory_client")

    current_state = rospy.wait_for_message('/joint_states', JointState, 2.0)
    joint_order = [JOINT_NAMES.index(name) for name in current_state.name if name in JOINT_NAMES]

    init_pos = [current_state.position[i] for i in joint_order]

    print(init_pos)

    init_vel = [0, 0, 0, 0, 0, 0]

    
    test_pos = copy(init_pos)
    test_pos[0] += 0.3

    print(test_pos)
    
    traj = Trajectory()
    rospy.on_shutdown(traj.stop)
    
    # simdata_dir = '../promp-ur5e-test/data/ur5e_kinesthetic_clf_cbf/'
    simdata1_f = datapath + 'tase-promp-pick_to_drop.txt'
    
    simdata1 = np.loadtxt(simdata1_f, delimiter=',')

    print(simdata1.shape)
    
    print("Adding points...")
    traj.add_point(init_pos, init_vel, 1.0)

    slow_time = np.linspace(2.0, 6500, len(simdata1))
    time_wait_start = 2.0 # seconds
    print(slow_time.shape)

    input("Press Enter to continue...")

    for i in range(len(simdata1)):
        if i % 200 == 0:
            print(simdata1[i, 0], simdata1[i, 1:7], simdata1[i, 7:])
            traj.add_point(simdata1[i,1:7], simdata1[i, 7:], simdata1[i, 0] + time_wait_start)

    # try to correct velocities slowing down prematurely
    traj.add_point(simdata1[-1, 1:7], simdata1[-1, 7:], simdata1[-1, 0] + time_wait_start + time_wait_start)

    input("Press Enter to continue...")


    #traj.add_point(test_pos, init_vel, 3.0)
    #traj.add_point(init_pos, init_vel, 6.0)

    try:
        print("Starting...")

        traj.start()

        print("Waiting...")
        traj.wait(5.0)
        # wait for manual human op
        print("Exiting...")
        exit()
    except Exception as e:
        print(e)
        exit(-1)
    

class Trajectory(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/pos_joint_traj_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.05)
        self._goal.trajectory.joint_names = copy(JOINT_NAMES)
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
        point.positions = copy(positions)
        point.velocities = copy(velocities)
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
