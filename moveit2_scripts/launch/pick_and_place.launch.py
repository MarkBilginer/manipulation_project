import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

MODE_FILE_DEFAULT = "/tmp/robot_env.flag"  # override with env var ROBOT_ENV_FILE

def _read_mode() -> str:
    """Read 'sim' or 'real' from the mode file. Defaults to 'real' if missing."""
    path = os.environ.get("ROBOT_ENV_FILE", MODE_FILE_DEFAULT)
    try:
        with open(path, "r") as f:
            return f.read().strip().lower()
    except Exception:
        return "real"

def _setup(context, *args, **kwargs):
    mode = _read_mode()                 # "sim" or "real"
    is_sim = (mode == "sim")

    # Choose MoveIt config package based on mode
    moveit_pkg = "my_moveit_config" if is_sim else "real_moveit_config"

    # NOTE: replace "name" with your robot name if needed.
    moveit_config = MoveItConfigsBuilder("name", package_name=moveit_pkg).to_moveit_configs()

    # Choose the executable built from your corresponding C++ file
    executable = "pick_and_place_sim" if is_sim else "pick_and_place_real"

    node = Node(
        name="pick_and_place_node",
        package="moveit2_scripts",
        executable=executable,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": is_sim},
        ],
    )

    msg = (f"[env-flag] mode='{mode}'. "
           f"Using pkg={moveit_pkg}, exec={executable}, use_sim_time={is_sim}.")
    return [LogInfo(msg=msg), node]

def generate_launch_description():
    # OpaqueFunction lets us compute mode & build the Node at launch time
    return LaunchDescription([OpaqueFunction(function=_setup)])
