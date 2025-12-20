from typing import Any

import numpy as np
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def is_array_like(obj: Any) -> bool:
    return isinstance(obj, (tuple, set, list, np.ndarray))


def flatten_dict(
    dictionary: dict, parent_key: str = "", sep: str = "/"
) -> dict[str, Any]:
    flattened = {}
    for key, value in dictionary.items():
        new_key = f"{parent_key}{sep}{key}" if parent_key else str(key)
        if isinstance(value, dict):
            flattened.update(flatten_dict(value, new_key, sep=sep))
        elif is_array_like(value):
            for index, item in enumerate(value):
                flattened.update(flatten_dict({index: item}, new_key, sep=sep))
        else:
            flattened[new_key] = value
    return flattened


def dict_to_diagnostics(
    dictionary: dict,
    level: int = DiagnosticStatus.OK,
    name: str = "",
    message: str = "",
    hardware_id: str = "",
) -> DiagnosticStatus:
    flattened = flatten_dict(dictionary)
    status = DiagnosticStatus(
        level=level,
        name=name,
        message=message,
        hardware_id=hardware_id,
    )

    for key, value in flattened.items():
        status.values.append(KeyValue(key=key, value=str(value)))

    return status
