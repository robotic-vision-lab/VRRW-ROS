## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['robotiq_ur_ros_wrapper', 'robotiq_modbus_server', 'robotiq_ros_controller'],
    package_dir={'': 'src',
                 'robotiq_modbus_server': 'src/robotiq_modbus_server',
                 'robotiq_ros_controller': 'src/robotiq_ros_controller'})

setup(**setup_args)