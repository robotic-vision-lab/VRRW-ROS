## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['custom_utilities',
              'ur_remote_dashboard',
              'robotiq_controller'],
    package_dir={'custom_utilities': 'src/custom_utilities',
                 'ur_remote_dashboard': 'src/ur_remote_dashboard',
                 'robotiq_controller': 'src/robotiq_controller'})

setup(**setup_args)