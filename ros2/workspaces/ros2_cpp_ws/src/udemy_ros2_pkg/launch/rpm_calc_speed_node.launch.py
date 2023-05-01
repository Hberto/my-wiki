from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="udemy_ros2_pkg", 
            executable="rpm_publisher",
            name="rpm_pub_node",
            parameters=[
                {"rpm_val": 5.0}
            ]
        ),
        Node(
            package="udemy_ros2_pkg", 
            executable="calc_speed",
            name="calc_speed_node",
            parameters=[
                {"wheel_radius": 10/100}
            ]
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/speed'],
            output='screen'
        )

    ])