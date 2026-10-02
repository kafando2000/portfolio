import os
from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess 
from ament_index_python.packages import get_package_share_directory 
from moveit_configs_utils import MoveItConfigsBuilder 
from launch.actions import RegisterEventHandler
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
import xacro

def generate_launch_description(): 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    use_sim_time = LaunchConfiguration('use_sim_time')
  

    # spwan controllers
    start_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'arm_controller','--'],
            output='screen')
    #start diff drive controller
    start_diff_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'diff_drive_controller','--'],
            output='screen')
    # sine_controller
    start_sine_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'sine_controller','--'],
            output='screen')
    #gripper controller
    start_gripper_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'mm_gripper_controller','--'],
            output='screen')

    # Launch joint state broadcaster
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
            output='screen')


    # Launch the joint state broadcaster after spawning the robot
    # load_joint_state_broadcaster_cmd = RegisterEventHandler(
    #    event_handler=OnProcessExit(
    #    target_action="",
    #    on_exit=[start_joint_state_broadcaster_cmd],))

    # Launch the arm controller after launching the joint state broadcaster
    load_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=start_joint_state_broadcaster_cmd,
        on_exit=[start_arm_controller_cmd],))
    # Launch the diff drive controller after launching the arm controller
    load_diff_drive_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=start_arm_controller_cmd,
        on_exit=[start_diff_drive_controller_cmd],))
    # Launch the sine controller after launching the diff drive controller
    load_sine_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=start_diff_drive_controller_cmd,
        on_exit=[start_sine_controller_cmd],))
    load_gripper_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=start_diff_drive_controller_cmd,
        on_exit=[start_gripper_controller_cmd],))
    
    """       
    # Launch the rqt_joint_trajectory_controller standalone
    rqt_traj_controller = ExecuteProcess(
            cmd=['rqt', '--standalone', 'rqt_joint_trajectory_controller'],
            output='screen',
        )
    # Delay the start by 7 seconds using TimerAction
    delayed_rqt_traj_controller = TimerAction(
        period=8.0,  # Delay in seconds
        actions=[rqt_traj_controller]
    )
    """  
    
    
    return LaunchDescription([
        #declare_use_sim_time_cmd,
        load_arm_controller_cmd,
        load_diff_drive_controller_cmd,
        start_joint_state_broadcaster_cmd,
        load_gripper_controller_cmd,
        ])
