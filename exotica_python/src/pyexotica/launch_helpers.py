from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import EmitEvent, RegisterEventHandler


def shutdown_on_exit(actions):
    """Return RegisterEventHandler actions that shut down the launch when any node exits."""
    event_handlers = []
    for n in actions:
        event_handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=n,
                    on_exit=[EmitEvent(event=Shutdown(reason=f"{n.name} exited"))]
                )
            )
        )
    return event_handlers
