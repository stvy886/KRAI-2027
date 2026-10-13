from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    enc_xr_node = Node(
        package="auto",
        executable="modbusxr"
    )

    enc_xl_node = Node(
        package="auto",
        executable="modbusxl"
    )

    enc_y_node = Node(
        package="auto",
        executable="modbusy"
    )   

    deadwheel_node = Node(
        package="auto",
        executable="deadwheel"
    )   

    odometry_node = Node(
        package="auto",
        executable="odom"
    )

    ld.add_action(enc_xr_node)
    ld.add_action(enc_xl_node)
    ld.add_action(enc_y_node)
    ld.add_action(deadwheel_node)
    ld.add_action(odometry_node)
    

    return ld