import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("WSTTA_ROBOT", package_name="mm_moveit")
        .robot_description()
        .robot_description_semantic(file_path="config/WSTTA_ROBOT.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution()
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        name="move_group",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "joint_state_topic": "/joint_states",
            }
        ],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    return LaunchDescription([
        move_group_node
    ])