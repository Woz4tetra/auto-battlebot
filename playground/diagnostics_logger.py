from diagnostic_msgs.msg import DiagnosticArray

from app.shared.messages.diagnostics.diagnostics_module_logger import (
    DiagnosticsModuleLogger,
)
from app.shared.messages.header import Header
from app.shared.net.messages.publisher import Publisher


class DiagnosticsLogger:
    """
    A singleton class that manages diagnostics loggers and publishes their statuses.
    Recommended initialization is through the register_diagnostics function.

    Example usage:
        diagnostics_logger = DiagnosticsLogger.get_logger('module_name')
        diagnostics_logger.info(message='This is an info message')
        diagnostics_logger.debug({'sensor_name': 'sensor_value'})
        diagnostics_logger.error(message='Hardware has failed', data={'error_code': 123})

    If a single logger sets multiple messages before being published, they are concatenated
    together in the final output.
    For example:
        diagnostics_logger.warning(message='Low battery')
        diagnostics_logger.error(message='Sensor disconnected')
    Results in a diagnostic message: "Low battery | Sensor disconnected".

    Nested dictionaries and lists/arrays are flattened with keys joined by slashes.
    For example:
        diagnostics_logger.error(data={'motor': {'temperatures': [95, 94, 90], 'voltage': 12.5}})
    Results shown in Foxglove:
        motor/temperature/0: 95
        motor/temperature/1: 94
        motor/temperature/2: 90
        motor/voltage: 12.5

    Call DiagnosticsLogger.publish() periodically to send accumulated diagnostics. Ideally from a
    single place at the top level of the application.
    """

    loggers: dict[str, DiagnosticsModuleLogger] = {}
    diagnostics_publisher: Publisher[DiagnosticArray] | None = None
    app_name: str = ""

    def __init__(
        self, app_name: str, diagnostics_publisher: Publisher[DiagnosticArray]
    ) -> None:
        cls = self.__class__
        if cls.diagnostics_publisher is not None:
            raise RuntimeError("DiagnosticsManager already initialized")
        cls.diagnostics_publisher = diagnostics_publisher
        cls.app_name = app_name

    @classmethod
    def is_initialized(cls) -> bool:
        return cls.diagnostics_publisher is not None

    @classmethod
    def get_logger(cls, name: str) -> DiagnosticsModuleLogger:
        if logger := cls.loggers.get(name):
            return logger
        logger = DiagnosticsModuleLogger(cls.app_name, name)
        cls.loggers[name] = logger
        return logger

    @classmethod
    def remove_logger(cls, name: str) -> None:
        if logger := cls.loggers.get(name):
            logger.clear()
            del cls.loggers[name]

    @classmethod
    def publish(cls) -> None:
        if cls.diagnostics_publisher is None:
            raise RuntimeError("DiagnosticsManager not initialized")
        statuses = []
        for logger in cls.loggers.values():
            if not logger.has_status():
                continue
            statuses.append(logger.get_status())
            logger.clear()
        if len(statuses) == 0:
            return
        diagnostics = DiagnosticArray(
            header=Header.auto().to_ros_msg(), status=statuses
        )
        cls.diagnostics_publisher.send(diagnostics)
