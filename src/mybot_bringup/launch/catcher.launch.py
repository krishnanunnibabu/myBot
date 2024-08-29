from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    turtle_sim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[
            {"background_b": 147},
            {"background_g": 189},
            {"background_r": 203}
        ]
    )

    mybot_controller_node = Node(
        package="mybot_controller",
        executable="catcher"
    )

    ld.add_action(turtle_sim_node)
    ld.add_action(mybot_controller_node)


    return ld