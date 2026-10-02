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
    # Build MoveIt 2 configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_name="WSTTA_ROBOT",package_name="mm_moveit",
                             robot_description="robot_description")
        .robot_description(file_path="config/WSTTA_ROBOT.urdf.xacro")
        .robot_description_semantic(file_path="config/WSTTA_ROBOT.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        ).to_moveit_configs()
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    moveit_pkg = get_package_share_directory('mm_moveit')
    mm_behaviors_pkg = get_package_share_directory('mm_behaviors')

    bt_node_yaml = os.path.join( 
        get_package_share_directory("mm_behaviors"), "config", "bt_parameters.yaml" 
    ) 

   
    tree_path = os.path.join(mm_behaviors_pkg, 'xml_tree', 'mm_behaviors.xml')
    
    # Main Behavior Tree node
    wstta_bt_node = Node( 
        name="wstta_bt_node", 
        package="mm_behaviors", 
        executable="wstta_bt_node", 
        output="screen", 
        arguments=['--ros-args', '--log-level', 'moveit:=debug'],
        parameters=[bt_node_yaml,{"tree_xml_file": tree_path},
                    moveit_config.to_dict()
                    ], 
    ) 

    return LaunchDescription([
        declare_use_sim_time_cmd,
        wstta_bt_node,
        ])



