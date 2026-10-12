from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_rsp_launch,
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_static_virtual_joint_tfs_launch,
    generate_spawn_controllers_launch,
)


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="real",
        description="ros2_control hardware mode: 'mock' or 'real'",
    )
    mode = LaunchConfiguration("mode")

    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so_arm_config")
        .robot_description(mappings={"mode": mode})
        .to_moveit_configs()
    )

    ld = LaunchDescription([mode_arg])

    for action in generate_static_virtual_joint_tfs_launch(moveit_config).entities:
        ld.add_action(action)

    for action in generate_rsp_launch(moveit_config).entities:
        ld.add_action(action)

    for action in generate_move_group_launch(moveit_config).entities:
        ld.add_action(action)

    for action in generate_moveit_rviz_launch(moveit_config).entities:
        ld.add_action(action)

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                str(moveit_config.package_path / "config/ros2_controllers.yaml")
            ],
            remappings=[
                ("/controller_manager/robot_description", "/robot_description")
            ],
            output="screen",
        )
    )

    startup_sequence_node = Node(
        package="so_arm_101_startup",
        executable="startup_sequence",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    ld.add_action(TimerAction(period=8.0, actions=[startup_sequence_node]))

    for action in generate_spawn_controllers_launch(moveit_config).entities:
        ld.add_action(action)

    return ld
