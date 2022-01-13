import rospy

from rvl_ur_remote_dashboard.URRemoteDashboard import URRemoteDashboard

rospy.init_node('ur_remote_dashboard_node', anonymous=True)

u = URRemoteDashboard()
u.get_safety_mode()
# u.release_brakes()
# u.power_off_arm()