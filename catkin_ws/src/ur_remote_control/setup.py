from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ur_remote_control',
              'ur_remote_dashboard',
              'robotiq_conroller'],
    package_dir={'': 'src',
                 'ur_remote_dashboard': 'src/ur_remote_dashboard',
                 'robotiq_controller': 'src/robotiq_controller'}
)

setup(**d)