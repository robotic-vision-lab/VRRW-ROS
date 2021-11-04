## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ur_remote_dashboard', 'ur_motion_planner'],
    package_dir={'': 'src',
                 'ur_motion_planner': 'src/ur_motion_planner'})

setup(**setup_args)