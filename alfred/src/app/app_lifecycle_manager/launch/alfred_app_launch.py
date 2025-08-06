# Copyright (c) 2024 Arnix Robotix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    ld = launch.LaunchDescription()

    app_controller_node = launch_ros.actions.LifecycleNode(
        name='app_controller', namespace='',
        package='app_controller', executable='app_controller_node', output='screen')

    register_event_handler_for_app_controller_reaches_configure_state = \
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=app_controller_node, goal_state='inactive',
                entities=[
                    launch.actions.LogInfo(
                        msg="app_controller_node active"),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(app_controller_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )

    emit_event_to_request_app_controller_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(app_controller_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ld.add_action(register_event_handler_for_app_controller_reaches_configure_state)
    ld.add_action(app_controller_node)
    ld.add_action(emit_event_to_request_app_controller_configure_transition)

    return ld
