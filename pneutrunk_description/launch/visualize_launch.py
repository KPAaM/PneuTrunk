import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
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

    return LaunchDescription([
        rviz_node,
        robot_state_publisher_node
    ])


# def generate_launch_description():
#     urdf_tutorial_path = get_package_share_path('pneutrunk_description')
#     default_model_path = urdf_tutorial_path / 'urdf/pneutrunk.xacro'

#     model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
#                                       description='Absolute path to robot urdf file')
    
#     robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
#                                        value_type=str)
    
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'robot_description': robot_description}]
#     )

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen'
#     )


#     return LaunchDescription([
#         model_arg,
#         rviz_node,
#         robot_state_publisher_node
#     ])


