from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    # Get path to xacro file
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'custom_arm.urdf.xacro')
    urdf_file = os.path.abspath(urdf_file)

    # Process xacro
    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toxml()

    return LaunchDescription([
        # Start Gazebo Fortress (empty world)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Delay a few seconds to allow sim to start
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='gz_spawn_entity',
                    output='screen',
                    arguments=[
                        '-name', 'custom_arm',
                        '-x', '0', '-y', '0', '-z', '1.0',
                        '-topic', 'robot_description',
                    ],
                    parameters=[{'robot_description': robot_desc}],
                )
            ]
        ),
    ])

