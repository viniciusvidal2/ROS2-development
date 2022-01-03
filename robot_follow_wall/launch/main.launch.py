from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_follow_wall', executable='find_wall_server_node', output='screen'),
        launch_ros.actions.Node(
            package='robot_follow_wall', executable='follow_wall', output='screen'),
        launch_ros.actions.Node(
            package='robot_follow_wall', executable='record_odom_action_node', output='screen'),
    ])