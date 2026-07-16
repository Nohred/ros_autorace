import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Rutas a los launch existentes
    detect_pkg = get_package_share_directory('turtlebot3_autorace_detect')
    mission_pkg = get_package_share_directory('turtlebot3_autorace_mission')

    ## LANE DETECTION AND CONTROL NODES
    detect_lane_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detect_pkg, 'launch', 'detect_lane.launch.py')
        )
    )

    control_lane_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mission_pkg, 'launch', 'control_lane.launch.py')
        )
    )

    
    ##  TRAFFIC LIGHT DETECTION NODE
    detect_light_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detect_pkg, 'launch', 'detect_traffic_light.launch.py')
        )
    )

    ## SIGN DETECTION LAUNCHES
    sign_intersection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detect_pkg, 'launch', 'detect_sign.launch.py')
        ),
        launch_arguments={
            'mission': 'intersection',
            'traffic_sign_topic': '/detect/sign_intersection',
        }.items()
    )

    sign_construction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detect_pkg, 'launch', 'detect_sign.launch.py')
        ),
        launch_arguments={
            'mission': 'construction',
            'traffic_sign_topic': '/detect/sign_construction',
        }.items()
    )

    sign_parking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detect_pkg, 'launch', 'detect_sign.launch.py')
        ),
        launch_arguments={
            'mission': 'parking',
            'traffic_sign_topic': '/detect/sign_parking',
        }.items()
    )

    sign_level_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detect_pkg, 'launch', 'detect_sign.launch.py')
        ),
        launch_arguments={
            'mission': 'level_crossing',
            'traffic_sign_topic': '/detect/sign_level',
        }.items()
    )

    sign_tunnel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detect_pkg, 'launch', 'detect_sign.launch.py')
        ),
        launch_arguments={
            'mission': 'tunnel',
            'traffic_sign_topic': '/detect/sign_tunnel',
        }.items()
    )

    level_crossing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detect_pkg, 'launch', 'detect_level_crossing.launch.py')
        )
    )
    

    avoid_construction_node = Node(
        package='turtlebot3_autorace_mission',
        executable='avoid_construction', # El nombre en tu setup.py
        name='avoid_construction',
        output='screen'
    )

    

    return LaunchDescription([
        detect_lane_launch,
        control_lane_launch,
        detect_light_launch,
        sign_intersection_launch,
        sign_construction_launch,
        sign_parking_launch,
        sign_level_launch,
        sign_tunnel_launch,
        level_crossing_launch,
        avoid_construction_node
    ])