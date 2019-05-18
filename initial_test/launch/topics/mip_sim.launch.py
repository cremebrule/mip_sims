"""Launch the MIP controller and plant nodes to run a simulation"""

import launch
import launch_ros.actions

def generate_launch_description():
    controller = launch_ros.actions.Node(
        package='initial_test', node_executable='mip_controller', output='screen')
    plant = launch_ros.actions.Node(
        package='initial_test', node_executable='mip_plant', output='screen',
	parameters=["/home/cremebrule/ros2_ws/src/ros2/initial_test/plant_params.yaml"])
    return launch.LaunchDescription([
        controller,
        plant,
        
        # Shutdown when controller exits
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=controller,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])
