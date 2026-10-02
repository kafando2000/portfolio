import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
def generate_move_group_launch(moveit_config):
    moveit_config = (
        MoveItConfigsBuilder("mm_moveit")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("mm_behaviors"),
                "config",
                "mm_moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )


    pkg_dir = get_package_share_directory('mm_behaviors')
    tree_path = os.path.join(pkg_dir, 'xml_tree', 'mm_behaviors.xml')
    # MoveItCpp demo executable

    moveit_cpp_node = Node(
        name="mm_behaviors",
        package="mm_behaviors",
        executable="wstta_bt_node",
        output="screen",
        parameters=[moveit_config.to_dict(),{"tree_xml_file":tree_path},{}],
    )

   

    ld = LaunchDescription()
    

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("use_sim_time", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )


    kinematics_yaml = load_yaml("mm_moveit", "config/kinematics.yaml")
    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    use_sim_time = LaunchConfiguration("use_sim_time")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,

    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        kinematics_yaml,
        {"tree_xml_file":tree_path}
        
    ]

    return ld
"""

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("WSTTA_ROBOT", package_name="mm_moveit").to_moveit_configs()
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    pkg_dir = get_package_share_directory('mm_behaviors')
    tree_path = os.path.join(pkg_dir, 'xml_tree', 'mm_behaviors.xml')
    ld = LaunchDescription()
    
    bt_node = Node(
            package='mm_behaviors',
            executable='wstta_robot_bt_node',
            name='wstta_robot_bt_node',
            output='screen',
            parameters=[moveit_config.to_dict(),{'tree_xml_file': tree_path.__str__()},{"use_sim_time":True}],
        )
    ld.add_action(bt_node)
    return ld





"""
