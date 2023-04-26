import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CONTROL_PKG = "rr1_control"

def generate_launch_description():
    # Read arguments and look for number_of_robots, else use default value 2
    # This cannot be declared as a Launch Argument because it neers to be available
    # before the LaunchDescription is executed
    number_of_robots = 2
    for arg in sys.argv:
        if arg.startswith("number_of_robots:="):
            number_of_robots = int(arg.split(":=")[1])

    # Create a LaunchDescription object
    ld = LaunchDescription()
    
    ros_tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
    )
    ld.add_action(ros_tcp_endpoint)

    # Spawn robot trajectory publishers
    for i in range(number_of_robots):
        rr1_ns = "rr1_{}".format(i)

        # Run test trajectory data publisher for given robot
        trajectory_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(CONTROL_PKG),
                    'launch/test_joint_trajectory_controller.launch.py',
                ])
            ]),
            launch_arguments={
                'topic': '/{}/joint_trajectory_controller/joint_trajectory'.format(rr1_ns)
            }.items()
        )
        ld.add_action(trajectory_publisher)

    return ld