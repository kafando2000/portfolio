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
    ros_gz_bridge_config = os.path.join(pkg_dir, 'config', 'ros_gz_bridge.yaml')

      #print("#########################################################################")
    # Start Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args" : PythonExpression(["'", world_file, " -r'"]),
                          'use_sim_time': use_sim_time

        }.items()
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
          'use_sim_time': use_sim_time
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
        parameters=[{'use_sim_time': use_sim_time}]
      )
    
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            gazebo,
            spawn,
            start_gazebo_ros_bridge_cmd,
           # start_gazebo_ros_image_bridge_cmd,
        ]
    )
    