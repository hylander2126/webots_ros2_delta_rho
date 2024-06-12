import os
import launch
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node


def generate_launch_description():
    package_dir = get_package_share_directory('delta_rho')
    
    # --- Define URDF robot; has to match the WEBOTS_CONTROLLER_URL of driver node ---
    xacro_path = os.path.join(package_dir, 'description', 'holonomic_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toprettyxml(indent='  ')

    # xacro_path = os.path.join(package_dir, 'description', 'epuck.urdf')
    # robot_description = xacro_path
    # spawn_URDF = URDFSpawner(
    #     name='holonomic_robot',
    #     urdf_path=xacro_path,
    #     translation='0, 0, 0',
    #     rotation='0, 0, 0, 1',
    # )

    spawn_URDF = URDFSpawner(
        name='holonomic_robot',
        robot_description=robot_description,
        relative_path_prefix=os.path.join(package_dir, 'description'),
        # box_collision=True,
        translation='0, 0, 0',
        rotation='0, 0, 0, 1',
    )

    # --- Define driver node ---
    robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'holonomic_robot'},
        parameters=[
            {'robot_description': xacro_path},
        ],
    )
    
    # The WebotsLauncher is a Webots custom action that allows you to start a Webots simulation instance.
    # It searches for the Webots installation in the path specified by the `WEBOTS_HOME` environment variable and default installation paths.
    # The Ros2Supervisor node is mandatory to spawn an URDF robot.
    # The accepted arguments are:
    # - `world` (str): Path to the world to launch.
    # - `gui` (bool): Whether to display GUI or not.
    # - `mode` (str): Can be `pause`, `realtime`, or `fast`.
    # - `ros2_supervisor` (bool): Spawn the `Ros2Supervisor` custom node that communicates with a Supervisor robot in the simulation.
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt'),
        ros2_supervisor=True
    )
    
    # -- What to run/launch ---
    return LaunchDescription([
        webots,
        webots._supervisor,
        spawn_URDF,
        # robot_driver,

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