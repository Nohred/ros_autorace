import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Obtener las rutas de los paquetes
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    camera_pkg = get_package_share_directory('turtlebot3_autorace_camera')

    # 1. Lanza el mundo de Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'turtlebot3_autorace_2020.launch.py')
        )
    )

    # 2. Lanza calibracion intrinseca
    intrinsic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_pkg, 'launch', 'intrinsic_camera_calibration.launch.py')
        )
    )

    # 3. Lanza calibracion extrinseca
    extrinsic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_pkg, 'launch', 'extrinsic_camera_calibration.launch.py')
        )
    )

    return LaunchDescription([
        # Arrancamos Gazebo de inmediato
        gazebo_launch,
        
        # Esperamos 5 segundos a que Gazebo cargue antes de lanzar la intrinseca
        TimerAction(
            period=10.0,
            actions=[intrinsic_launch]
        ),
        
        # Esperamos 7 segundos (2 seg despues de la intrinseca) para la extrinseca
        TimerAction(
            period=5.0,
            actions=[extrinsic_launch]
        )
    ])