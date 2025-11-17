from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("deplacement_test_1"),
            "config",
            "test_goal_publishers_config.yaml",
        ]
    )

    check_starting_point = LaunchConfiguration("check_starting_point")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "check_starting_point",
                default_value="true",
                description="Verify that the robot is at a preconfigured pose in order to avoid large unexpected motions.",
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_scaled_joint_trajectory_controller",
                parameters=[
                    position_goals,
                    {
                        "check_starting_point": check_starting_point,
                    },
                ],
                output="screen",
            ),
        ]
    )
