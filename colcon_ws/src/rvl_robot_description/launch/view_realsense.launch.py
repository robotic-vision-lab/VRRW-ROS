import os
import xacro
import tempfile
import launch
import sys

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.dirname(os.path.realpath(__file__)))

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path

def generate_launch_description():
    available_urdf_files = [f for f in os.listdir(os.path.join(get_package_share_directory('rvl_robot_description'), 'xacro', 'intel_realsense')) if f.startswith('test_')]

    params = dict([aa for aa in [aa.split(':=') for aa in sys.argv] if len(aa)==2])

    if ('model' not in params or params['model'] not in available_urdf_files):
        print ('USAGE:')
        print ('ros2 launch rvl_robot_description view_model.launch.py model:=<model>')
        print ('Available argumants for <model> are as follows:')
        print ('\n'.join(available_urdf_files))
        return launch.LaunchDescription()

    rviz_config_dir = os.path.join(get_package_share_directory('rvl_robot_description'), 'rviz', 'view_realsense.rviz')

    xacro_path = os.path.join(get_package_share_directory('rvl_robot_description'), 'xacro', 'intel_realsense', params['model'])

    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true'})

    rviz_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
    )

    model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments = [urdf]
    )

    return launch.LaunchDescription([rviz_node, model_node])