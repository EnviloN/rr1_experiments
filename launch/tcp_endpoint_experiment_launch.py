from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

UNITY = False
# PAYLOAD_SIZE = 2
# TIMER_DELTA = 200
MSG_CNT = 1000
LOGFILE = "log/log.csv"

def generate_launch_description():
    payload_size_value = LaunchConfiguration('payload_size')
    time_delta_value = LaunchConfiguration('time_delta')

    payload_size_launch_arg = DeclareLaunchArgument(
            'payload_size',
            default_value='2'
        )
    time_delta_launch_arg = DeclareLaunchArgument(
            'time_delta',
            default_value='200'
        )

    subscriber_node = Node(
            package='rr1_experiments',
            executable='experiment_subscriber_node',
            name='experiment_subscriber',
            parameters=[
                {'payload_size': payload_size_value},
                {'unity': UNITY},
                {'message_count': MSG_CNT},
                {'logfile': LOGFILE}
            ]
        )
    
    publisher_node = Node(
            package='rr1_experiments',
            executable='experiment_publisher_node',
            name='experiment_publisher',
            parameters=[
                {'timer_delta_ms': time_delta_value},
                {'message_count': MSG_CNT},
                {'payload_size': payload_size_value}
            ]
        )
    
    ros_tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
    )

    nodes = [payload_size_launch_arg, time_delta_launch_arg, publisher_node]
    if UNITY:
        nodes.append(ros_tcp_endpoint)
    else:
        nodes.append(subscriber_node)

    return LaunchDescription(nodes)
