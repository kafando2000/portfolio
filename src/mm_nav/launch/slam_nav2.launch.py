import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    mm_nav2_pkg = get_package_share_directory('mm_nav')
    pkg_slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': os.path.join(mm_nav2_pkg, 'params', 'nav2_params.yaml'),
            #'map_file': os.path.join(mm_nav2_pkg, 'maps', 'map_02.yaml'),
            'package_path': mm_nav2_pkg, 
        }.items()
    )
    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('mm_nav'),
                'rviz',
                'nav2_default_view.rviz'
            )
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    
    #static_transform_publisher_node = Node(package='tf2_ros', executable='static_transform_publisher',   name='map_to_odom',
      #  output='screen',
       # arguments=['0.0', '0.0', '0', '0', '0', '0', 'map', 'odom'],
     #   parameters=[{'use_sim_time': True}]
   # )

    # Include slam_toolbox launch file
    """
    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(mm_nav2_pkg, 'params', 'slam_toolbox_async.yaml'),
        }.items()
    )
    """
    
    slam_toolbox_launch_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('slam_toolbox'),
            'launch', 'localization_launch.py'
    )),
    launch_arguments={
        'slam_params_file': os.path.join(mm_nav2_pkg, 'params', 'slam_toolbox_async.yaml'),
        #'map_file': os.path.join(mm_nav2_pkg, 'maps', 'map.yaml'),
        'use_sim_time': 'true'
    }.items()
)


    ld = LaunchDescription()

    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    #ld.add_action(static_transform_publisher_node)
    #ld.add_action(slam_toolbox_launch_cmd)


    return ld
