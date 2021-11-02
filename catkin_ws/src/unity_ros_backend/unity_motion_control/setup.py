## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['robotiq_controller', 'ur_dashboard_wrapper', 'ur_moveit_wrapper'],
    package_dir={
        '': 'src',
        'robotiq_controller': 'src/robotiq_controller',
        'ur_dashboard_wrapper': 'src/ur_dashboard_wrapper',
        'ur_moveit_wrapper': 'src/ur_moveit_wrapper',
    },
    requires=['rospy', 'python3-modbus']
)

setup(**setup_args)