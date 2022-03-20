from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    talker_node = Node(
        package="run_moveit_cpp",
        executable="ur5_run_move_group",
    )
    ld.add_action(talker_node)
    return ld
