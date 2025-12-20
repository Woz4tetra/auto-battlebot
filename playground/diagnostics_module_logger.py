from diagnostic_msgs.msg import DiagnosticStatus

from app.shared.messages.diagnostics.dict_to_diagnostics import dict_to_diagnostics


class DiagnosticsModuleLogger:
    """
    A logger for a specific module that accumulates diagnostic information. This should
    not be used directly; instead, use DiagnosticsLogger to manage multiple module loggers.
    """

    def __init__(self, app_name: str, logger_name: str) -> None:
        self.app_name = app_name
        self.logger_name = logger_name
        self.data: dict = {}
        self.level = DiagnosticStatus.OK
        self.messages: list[str] = []
        self._logged_messages: set[str] = set()
        self.clear()

    def log(self, level: int, data: dict | None = None, message: str = "") -> None:
        self.level = max(level, self.level)
        if message and message not in self._logged_messages:
            self._logged_messages.add(message)
            self.messages.append(message)
        if data:
            self.data.update(data)

    def debug(self, data: dict | None = None, message: str = "") -> None:
        self.log(DiagnosticStatus.OK, data, message)

    def info(self, data: dict | None = None, message: str = "") -> None:
        self.log(DiagnosticStatus.OK, data, message)

    def warning(self, data: dict | None = None, message: str = "") -> None:
        self.log(DiagnosticStatus.WARN, data, message)

    def error(self, data: dict | None = None, message: str = "") -> None:
        self.log(DiagnosticStatus.ERROR, data, message)

    def clear(self) -> None:
        self.level = DiagnosticStatus.OK
        self.messages = []
        self.data = {}
        self._logged_messages = set()

    def has_status(self) -> bool:
        return len(self.messages) > 0 or len(self.data) > 0

    def get_status(self) -> DiagnosticStatus:
        message = " | ".join(self.messages) if self.messages else ""
        return dict_to_diagnostics(
            self.data, self.level, self.logger_name, message, self.app_name
        )
