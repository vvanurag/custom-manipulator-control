from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    # Get URDF from xacro
    xacro_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'custom_arm.urdf.xacro'
    )
    xacro_file = os.path.abspath(xacro_file)
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Write to temp file (required by spawn_entity)
    urdf_tmp_path = '/tmp/custom_arm.urdf'
    with open(urdf_tmp_path, 'w') as f:
        f.write(robot_desc)

    return LaunchDescription([
        # Start Gazebo Fortress
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
            output='screen'
        ),

        # Spawn the robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', urdf_tmp_path, '-name', 'custom_arm'],
            output='screen'
        )
    ])

