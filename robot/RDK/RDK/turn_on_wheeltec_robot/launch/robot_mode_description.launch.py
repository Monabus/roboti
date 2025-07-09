import os
from pathlib import Path
import launch_ros.actions
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,LogInfo,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
#mmmmmmmmmmmmmmmec

    mini_mec = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_mec_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.0 ', '0', '0.18','0', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.195', '0', '0.35','0', '0','0','base_footprint','camera_link'],),
    ])


#dddddddddddddddddddiff

    mini_tank = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_diff_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.00', '0', '0.155','3.1415', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.14', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    mini_4wd = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_diff_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.0', '0', '0.155','0', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.195', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    mini_diff = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_diff_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.0', '0', '0.155','3.1415', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.0s', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

 
    # Create the launch description and populate
    ld = LaunchDescription()

    #Please select your car model here, the options are:
 
    #mini_diff, mini_4wd, mini_tank , mini_mec
    ld.add_action(mini_mec)
    return ld

