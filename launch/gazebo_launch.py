import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

EXPERIMENTS_PKG = "rr1_experiments"
GAZEBO_PKG = "gazebo_ros"

WORLD_FILE = "experiment"

def generate_launch_description():
    # ------------------------------- Fetch paths ------------------------------
    # Package share directories
    experiments_pkg = get_package_share_directory(EXPERIMENTS_PKG)
    gazebo_pkg = get_package_share_directory(GAZEBO_PKG)

    # --------------------- Include gazebo launch description ------------------
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    launch_argument = DeclareLaunchArgument(
        'world',
        default_value=["/home/envilon/dev_ws/src/rr1_experiments/worlds/experiment", ''],
        description='SDF world file'
    )
    paused = DeclareLaunchArgument(
        'pause',
        default_value=["true"],
        description='Pause simulation'
    )

    lockstep = DeclareLaunchArgument(
        'lockstep',
        default_value=["true"],
        description='lockstep simulation'
    )

    return LaunchDescription([            
            launch_argument,
            paused,
            lockstep,
            launch_gazebo
        ])
