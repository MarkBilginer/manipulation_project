import os
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from launch_ros.actions import Node

MODE_FILE_DEFAULT = "/tmp/robot_env.flag"  # override with env var ROBOT_ENV_FILE

def _write_sim_flag(context, *args, **kwargs):
    path = os.environ.get("ROBOT_ENV_FILE", MODE_FILE_DEFAULT)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("sim\n")
    return [LogInfo(msg=f"[env-flag] wrote '{path}' = sim")]

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )

    # Ensure we write the flag before starting Move Group
    return LaunchDescription([
        OpaqueFunction(function=_write_sim_flag),
        move_group_node,
    ])
