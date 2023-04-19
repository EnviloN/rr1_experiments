from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

GAZEBO_PKG = "gazebo_ros"
EXPERIMENTS_PKG = "rr1_experiments"
WORLD_FILE = "experiment"

def generate_launch_description():
    # --------------------- Include gazebo launch description ------------------
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(GAZEBO_PKG), 'launch', 'gazebo.launch.py'
            ])
        )
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=[PathJoinSubstitution([FindPackageShare(EXPERIMENTS_PKG), 'worlds', WORLD_FILE])],
        description='SDF world file'
    )
    
    paused_arg = DeclareLaunchArgument(
        'pause',
        default_value=["true"],
        description='Pause simulation'
    )

    return LaunchDescription([            
            world_arg,
            paused_arg,
            launch_gazebo
        ])
