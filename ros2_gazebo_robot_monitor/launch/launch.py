from launch import LaunchDescription
from launch_ros.actions import Node
from datetime import datetime

def generate_launch_description():
    # Construct the directory path with the current date and time
    directory = "/home/rexilius/workspace/ros_workspace/data_results/"
    date_code = datetime.now().strftime("%Y-%m-%d__%H-%M-%S")
    directory = directory + date_code + "/"

    # Create the nodes with the directory parameter
    m_robot_node = Node(
        package='ros2_gazebo_robot_monitor',
        executable='monitor_robot_node',
        name='monitor_robot_node',
        output='screen',
        parameters=[
            {'directory': directory},
        ]
    )

    m_computer_node = Node(
        package='ros2_gazebo_robot_monitor',
        executable='monitor_computer_node',
        name='monitor_computer_node',
        output='screen',
        parameters=[
            {'directory': directory},
        ]
    )

    return LaunchDescription([
        m_robot_node,
        m_computer_node,
    ])
