import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    RViz + robot model and urdf
    """
    pneutrunk_xacro_file = os.path.join(get_package_share_directory('pneutrunk_description'), 'urdf',
                                     'pneutrunk.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', pneutrunk_xacro_file])
    
    rviz_file = os.path.join(get_package_share_directory('pneutrunk_description'), 'config',
                             'visualize.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['--display-config', rviz_file]
    )
    continuum_model_node = Node(
        package='pneutrunk_tools',
        executable='pneutrunk_model',
        name='pneutrunk_model',
        output='screen',
    )

    """
    GUI
    """
    gui_node = Node(
        package='pneutrunk_gui',
        executable='pneutrunk_gui',
        name='pneutrunk_gui',
        output='screen',
    )

    """
    Cameras node
    """
    camera_gesture_node = Node(
        package='pneutrunk_gesture_control',
        executable='pneutrunk_gesture_control',
        name='pneutrunk_gesture_control',
        output='screen',
    )
    camera_object_detect_node = Node(
        package='pneutrunk_object_detection',
        executable='pneutrunk_object_detection',
        name='pneutrunk_object_detection',
        output='screen',
    )

    

    return LaunchDescription([
        rviz_node,
        robot_state_publisher_node,
        continuum_model_node,
        gui_node
        # camera_gesture_node,
        # camera_object_detect_node,
    ])

