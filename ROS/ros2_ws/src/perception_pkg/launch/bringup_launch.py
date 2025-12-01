# bringup_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Optional: allow overriding node names or topics via launch args
    use_screen_output = LaunchConfiguration('screen', default='true')

    camara_node = Node(
        package='perception_pkg',
        executable='camara',
        name='camara',
        output='screen',
        emulate_tty=True,  # nicer formatting on terminal
        parameters=[],
        remappings=[],
    )

    control_node = Node(
        package='control_pkg',
        executable='control',
        name='control',
        output='screen',
        emulate_tty=True,
    )

    coms_node = Node(
        package='coms_pkg',
        executable='coms',
        name='coms',
        output='screen',
        emulate_tty=True,
    )

    ld = LaunchDescription()

    # Example launch argument (not strictly necessary, but useful template)
    ld.add_action(DeclareLaunchArgument('screen', default_value='true',
                                        description='Print node output to screen'))

    # Add nodes
    ld.add_action(camara_node)
    ld.add_action(control_node)
    ld.add_action(coms_node)

    return ld
