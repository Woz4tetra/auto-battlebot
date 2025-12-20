import punq
from diagnostic_msgs.msg import DiagnosticArray

from app.shared.enums.topics import Topics
from app.shared.messages.diagnostics.diagnostics_logger import DiagnosticsLogger
from app.shared.net.ros.ros_factory import RosFactory


def initialize_diagnostics(container: punq.Container) -> None:
    if DiagnosticsLogger.is_initialized():
        return
    ros_factory: RosFactory = container.resolve(RosFactory)
    diagnostics_publisher = ros_factory.make_publisher(
        Topics.DIAGNOSTICS, DiagnosticArray, queue_size=10
    )
    DiagnosticsLogger(
        ros_factory.name, diagnostics_publisher
    )  # initialize shared class
