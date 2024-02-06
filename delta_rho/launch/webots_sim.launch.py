"""Launch Webots simulation."""

import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

import xacro
import os
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from launch_ros.actions import Node


PACKAGE_NAME = 'delta_rho'

def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration('world')

    # WebotsLauncher is a Webots custom action, allows starting a Webots sim instance. Ros2Supervisor node mandatory to spawn URDF.
    # Arguments are:
    # - `world` (str): Path to the world to launch.
    # - `gui` (bool): Whether to display GUI or not.
    # - `mode` (str): Can be `pause`, `realtime`, or `fast`.
    # - `ros2_supervisor` (bool): Spawn the `Ros2Supervisor` custom node that communicates with a Supervisor robot in the simulation.
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )


    # Define your URDF robots here
    # The name of an URDF robot has to match the WEBOTS_ROBOT_NAME of the driver node
    # You can specify the URDF file to use with "urdf_path"
    xacro_path = os.path.join(package_dir, 'description', 'holonomic_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml() # prettyxml(indent='  ')    

    spawn_URDF = URDFSpawner(
        name='holonomic_robot',
        robot_description=robot_description,
        relative_path_prefix=os.path.join(package_dir, 'description'),
    )

    # Define your robot driver here
    # WebotsController is a Webots custom action, allows starting a Webots driver node.
    ros2_control_params = os.path.join(package_dir, 'config', 'holonomic_drive.yaml')
    robot_driver = WebotsController(
        robot_name='holonomic_robot',
        # namespace='holonomic_robot',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
            # {'webots_ros2_driver': },
            ros2_control_params
        ],
    )

    # TEMP CONTROL NODE TO FIX MISSING /CONTROLLER_MANAGER ISSUE
    # control_node = Node(
	# 	package='controller_manager',
	# 	executable='ros2_control_node',
	# 	parameters=[ros2_control_params], # DO NOT USE 'robot_description', defaults to /robot_description topic
	# 	output='both'
	# )

    # Additional nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager', # 'holonomic_robot/controller_manager',
                   '--controller-manager-timeout', '100']
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager', # 'holonomic_robot/controller_manager',
                   '--controller-manager-timeout', '100']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        # namespace='holonomic_robot',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description, # '<robot name=""><link name=""/></robot>',
        }]
    )


    # -- What to run/launch ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='my_world.wbt',
            description='Choose a world file to launch'
        ),
        # Start webots and ros2supervisor
        webots,
        webots._supervisor,
        
        # Request URDF spawn
        spawn_URDF,

        # Other ROS2 nodes
        robot_state_publisher,
        # joint_state_broadcaster_spawner,
        # joint_trajectory_controller_spawner,
        
        # Launch the driver node once the URDF is spawned
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_URDF,
                on_stdout=lambda event: get_webots_driver_node(event, robot_driver),
            )
        ),

        # Kill all nodes once webots is closed
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])