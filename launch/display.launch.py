from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
import os
import xacro

def generate_launch_description():
    #urdf_path = os.path.join(
    #    os.getenv('AMENT_PREFIX_PATH').split(':')[0], 
    #    'share', 'custom_arm', 'urdf', 'custom_arm.urdf.xacro'
    #)
    urdf_path = os.path.join(
	    get_package_share_directory('custom_arm'),
	    'urdf',
	    'custom_arm.urdf.xacro'
	)
    '''
    urdf_path = PathJoinSubstitution([
	    FindPackageShare("custom_arm"),
	    "urdf",
	    "custom_arm.urdf.xacro"
	])
    urdf_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share',
        'custom_arm',
        'urdf',
        'custom_arm.urdf.xacro'
    )
    urdf_path = PathJoinSubstitution([
        FindPackageShare("custom_arm"),
        "urdf",
        "custom_arm.urdf.xacro"
    ])
    '''
    #print("urdf path: ",urdf_path)
    print("Rviz path: ", os.path.join(os.getenv('AMENT_PREFIX_PATH').split(':')[0],'custom_arm', 'rviz', 'view_saved.rviz'));
    doc = xacro.process_file(urdf_path)
    robot_description_config = doc.toprettyxml(indent='  ')
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            #parameters=[{'robot_description': open(urdf_path).read()}],
            #parameters=[{'robot_description': Command(['xacro', urdf_path])}],
            #parameters=[{
            #    'robot_description': Command([
            #        'xacro',
            #        PathJoinSubstitution([
            #            FindPackageShare('custom_arm'),
            #            'urdf',
            #            'custom_arm.urdf.xacro'
            #        ])
            #    ])
            #}],
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                os.getenv('AMENT_PREFIX_PATH').split(':')[0], 
                'share','custom_arm', 'rviz', 'view_saved.rviz')],
        )
    ])
    
    
'''
## This worked. SO saving it here

from launch import LaunchDescription
from launch_ros.actions import Node
import os

import xacro  # Use xacro Python module directly

def generate_launch_description():
    # Locate file using installed path
    urdf_file = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share',
        'custom_arm',
        'urdf',
        'custom_arm.urdf.xacro'
    )

    # Parse and process the xacro file
    doc = xacro.process_file(urdf_file)
    robot_description_config = doc.toprettyxml(indent='  ')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
'''
'''
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    # Absolute path to the installed xacro file
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'custom_arm.urdf.xacro'
    )
    urdf_path = os.path.abspath(urdf_path)

    # Process the Xacro file
    doc = xacro.process_file(urdf_path)
    robot_description_config = doc.toprettyxml(indent='  ')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                os.path.dirname(__file__), '..', 'rviz', 'view.rviz')],
            output='screen'
        )
    ])
'''
