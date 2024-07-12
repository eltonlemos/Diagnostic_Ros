"""Launch analyzer loader with parameters from yaml."""

import launch
import launch_ros.actions

analyzer_params_filepath = "/home/icepeng/catkin_ws/src/diagnostic_moog/src/analyzers.yaml"
# add_analyzer_params_filepath = "@ADD_ANALYZER_PARAMS_FILEPATH@"


def generate_launch_description():
    aggregator = launch_ros.actions.Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        output='screen',
        parameters=[analyzer_params_filepath])
    # add_analyzer = launch_ros.actions.Node(
    #     package='diagnostic_aggregator',
    #     executable='add_analyzer',
    #     output='screen',
    #     parameters=[add_analyzer_params_filepath]
    # )
    diag_publisher = launch_ros.actions.Node(
        package='diagnostic_moog',
        executable='diagnostic.py')
    return launch.LaunchDescription([
        aggregator,
        # add_analyzer,
        diag_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=aggregator,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )),
    ])