 
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
    # Package Directories
    moveit_pkg = get_package_share_directory('mm_moveit')
    # Parse robot description from xacro
    robot_description_file = os.path.join(moveit_pkg, 'config', 'WSTTA_ROBOT.urdf.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description,{'use_sim_time': use_sim_time}],
    )
    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher
        ])