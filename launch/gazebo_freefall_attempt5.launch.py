from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    # Resolve URDF path
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'custom_arm.urdf.xacro'
    )
    urdf_file = os.path.abspath(urdf_file)

    # Process the Xacro file
    doc = xacro.process_file(urdf_file)
    robot_description = doc.toprettyxml(indent='  ')

    # Save as temp .urdf to pass to gazebo
    urdf_path = '/tmp/custom_arm.urdf'
    with open(urdf_path, 'w') as f:
        f.write(robot_description)

    return LaunchDescription([
        # Gazebo Fortress simulator
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
            output='screen'
        ),

        # Spawn the robot in Gazebo using ros_gz
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_custom_arm',
            arguments=[
                '-file', urdf_path,
                '-name', 'custom_arm',
                '-x', '0', '-y', '0', '-z', '0.2'
            ],
            output='screen'
        )
    ])

