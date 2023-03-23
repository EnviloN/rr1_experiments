from launch import LaunchDescription
from launch_ros.actions import Node

UNITY = True

def generate_launch_description():
    server_node = Node(
            package='rr1_experiments',
            executable='experiment_server_node',
            name='experiment_server'
        )

    client_node = Node(
            package='rr1_experiments',
            executable='experiment_client_node',
            name='experiment_client',
            parameters=[
                {'timer_delta_ms': 500},
                {'message_count': 10},
                {'max_payload': 1048576},
                {'unity': UNITY},
                {'logfile': 'log/log.csv'},
            ]
        )
    
    ros_tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
    )

    nodes = [client_node]
    if UNITY:
        nodes.append(ros_tcp_endpoint)
    else:
        nodes.append(server_node)

    return LaunchDescription(nodes)
