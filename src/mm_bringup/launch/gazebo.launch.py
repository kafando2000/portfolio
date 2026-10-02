import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from os.path import join

def generate_launch_description():
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_dir = get_package_share_directory('mm_bringup')
    moveit_pkg = get_package_share_directory('mm_moveit')
    world_file = LaunchConfiguration("world_file", default = join(pkg_dir, "worlds", "mm_world.world"))

    # Parse robot description from xacro
    robot_description_file = os.path.join(moveit_pkg, 'config', 'WSTTA_ROBOT.urdf.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    ros_gz_bridge_config = os.path.join(pkg_dir, 'config', 'ros_gz_bridge.yaml')
    
    # Start Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description,{'use_sim_time': use_sim_time}],
    )
    #print("#########################################################################")
    # Start Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args" : PythonExpression(["'", world_file, " -r'"]),
                          'use_sim_time': use_sim_time

        }.items()
    )

    # Start RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'gazebo.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn Robot in Gazebo   
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'name': 'WSTTA_ROBOT',
                     'z':0.01,
                    'topic': 'robot_description',
                    'use_sim_time': use_sim_time}],
        output='screen',
    )


      # Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
          'config_file': ros_gz_bridge_config,
          'use_sim_time': True
        }],
        output='screen'
      )  
    # Start bridge fo cameras    
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
          '/camera/depth_image',
          '/camera/image',
        ],
        remappings=[
          ('/camera/depth_image', '/camera/depth/image_rect_raw'),
          ('/camera/image', '/camera/color/image_raw'),
        ],
        parameters=[{'use_sim_time': True}]
      )
    

    # Start arm controller
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
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=spawn ,
        on_exit=[start_joint_state_broadcaster_cmd],))

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




    return LaunchDescription(
        [
            # Nodes and Launches
            declare_use_sim_time_cmd,
            gazebo,
            spawn,
            start_gazebo_ros_bridge_cmd,
            start_gazebo_ros_image_bridge_cmd,
            robot_state_publisher,
            load_arm_controller_cmd,
            load_diff_drive_controller_cmd,
            load_joint_state_broadcaster_cmd,
            load_gripper_controller_cmd,
            #load_sine_controller_cmd,
            #rviz,
            #delayed_rqt_traj_controller,

        ]
    )