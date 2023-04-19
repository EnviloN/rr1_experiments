import sys, math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

GAZEBO_PKG = "rr1_gazebo"
DESCRIPTION_PKG = "rr1_description"
CONTROL_PKG = "rr1_control"

ROBOT_NAME = "rr1"
URDF_FILE = f"{ROBOT_NAME}.urdf.xacro"

def generate_robot_positions(cnt):

    num_rows = math.ceil(math.sqrt(cnt))
    num_cols = math.ceil(cnt / num_rows)

    coordinates = []
    for row in range(num_rows):
        for col in range(num_cols):
            if len(coordinates) == cnt:
                break
            coordinates.append((col, row))

    return coordinates

def generate_launch_description():
    # Specify the location of the URDF file
    urdf_path = PathJoinSubstitution([FindPackageShare(DESCRIPTION_PKG), "urdf", URDF_FILE])

    # Read arguments and look for number_of_robots, else use default value 2
    # This cannot be declared as a Launch Argument because it neers to be available
    # before the LaunchDescription is executed
    number_of_robots = 2
    for arg in sys.argv:
        if arg.startswith("number_of_robots:="):
            number_of_robots = int(arg.split(":=")[1])

    robot_positions = generate_robot_positions(number_of_robots)

    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Start Gazebo
    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(GAZEBO_PKG),
                    'launch/gazebo.launch.py'
                ])
            ])
        )
    ld.add_action(gazebo_launch)

    # Spawn all robots
    for i in range(number_of_robots):
        rr1_ns = "rr1_{}".format(i)
        xacro_command = Command(['xacro ', urdf_path, " ns:={}".format(rr1_ns)])

        # Start publishing robot description
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            namespace=rr1_ns,
            parameters=[{
                'use_sim_time': True,
                'robot_description': xacro_command}]
        )
        ld.add_action(robot_state_publisher)

        # Spawn robot instance in Gazebo
        gazebo_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=rr1_ns + '_spawn_entity',
            arguments=['-entity', rr1_ns,
                       '-x', str(robot_positions[i][0]), '-y', str(robot_positions[i][1]), '-z', '0',
                       '-topic', '/{}/robot_description'.format(rr1_ns)],
            output='screen'
        )
        ld.add_action(gazebo_node)

        # Actovate the joint trajectory controller
        joint_trajectory_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/{}/controller_manager".format(rr1_ns)]
        )
        ld.add_action(joint_trajectory_controller)

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