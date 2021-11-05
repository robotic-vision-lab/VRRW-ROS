#!/usr/bin/python3

from enum import auto
from math import radians, degrees
from moveit_commander.move_group import MoveGroupCommander
import rospy

# from backend_support.URDashboard import URDashboard
# from backend_support.RobotiqGripperController import Robotiq2FController

from ur_remote_dashboard.URDashboard import URDashboard

import moveit_commander as mc

installation_name = 'jerry_remote.installation'
program_name      = 'jerry_ext.urp'

locations = {
    'home' : [0.0, radians(-45.0), radians(-135.0), radians(-90.0), 0.0, radians(90.0)],
    'pick' : [radians(-91.63), radians(-125.0), radians(-82.9), radians(-60.05), radians(95.12), radians(0.19)],
    'near_pick' : [radians(-94.03), radians(-118.53), radians(-73.63), radians(-76.65), radians(95.08), radians(.19)]
}

class DumbCommander:
    def __init__(self):
        self.group:MoveGroupCommander = mc.MoveGroupCommander('manipulator')
        self.group.set_planning_time(10.0)
        self.group.set_num_planning_attempts(10)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.05)
        self.group.allow_looking(True)
        self.group.allow_replanning(True)
        
    def goto_joint_angle(self, angle):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[:] = angle[:]
        self.group.go(joint_goal, wait=True)
        self.group.stop()

if __name__ == '__main__':
    rospy.init_node('UR_ROS_dashboard_node', anonymous=True)

    dashboard = URDashboard()
    dashboard.stop_program()
    
    # dashboard.load_installation(installation_name)
    # dashboard.cold_boot()
    # dashboard.load_program(program_name)
    # dashboard.start_program()
    
    # rospy.sleep(rospy.Duration(2))
    # controller = Robotiq2FController(autoconnect=True)
    # controller.reset_gripper()
    # controller.activate_gripper()
    # controller.autoopen()
    
    # input('Press any key to continue...')
    
    # dc = DumbCommander()
    
    # while not rospy.is_shutdown():
    #     dc.goto_joint_angle(locations['home'])
    #     controller.autoopen()
    #     input('Press any key to continue...')
    #     dc.goto_joint_angle(locations['near_pick'])
    #     input('Press any key to continue...')
    #     dc.goto_joint_angle(locations['pick'])
    #     rospy.sleep(rospy.Duration(2))
    #     dashboard.unlock_protective_stop()
    #     controller.autoclose(255)
    #     rospy.sleep(rospy.Duration(2))
    #     dc.goto_joint_angle(locations['near_pick'])
    #     input('Press any key to continue...') 
    
    # joint_goal = group.get_current_joint_values()

    
    # input('Press any key to continue...')
    
    # group.execute(traj)
    
    # input('Press any key to continue...')
    
    # controller = Robotiq2FController(autoconnect=True)
    
    # input('Press any key to continue...')

    # controller.activate_gripper()
    # controller.autoclose(force=255)

    
    # controller.reset_gripper()
    # controller.activate_gripper()
    
    # input('Press any key to continue...')
    
    # while not rospy.is_shutdown():
        # controller.print_status()
        # input('Press any key to continue...')
        
        # controller.autoclose(force=25)
        # input('Press any key to continue...')
        
        # controller.autoopen(force=25)
        # input('Press any key to continue...')
        