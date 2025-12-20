import string
from typing import Any

from diagnostic_msgs.msg import DiagnosticStatus


def is_key_index(key: str) -> bool:
    return key.isdigit() or all(char in string.digits for char in key)


def try_float(key: str) -> float | None:
    try:
        return float(key)
    except ValueError:
        return None


def try_int(key: str) -> int | None:
    try:
        return int(key)
    except ValueError:
        return None


def unflatten_dict(dictionary: dict, sep: str = "/") -> dict:
    unflattened: dict[str, Any] = {}
    subkeys: dict[str, list[tuple[str, str]]] = {}
    for key, value in dictionary.items():
        if isinstance(key, str) and sep in key:
            first_key, remaining_key = key.split(sep, 1)
            subkeys.setdefault(first_key, []).append((key, remaining_key))
        else:
            unflattened[key] = value

    if len(subkeys) > 0:
        for first_key, remaining_data in subkeys.items():
            sub_dict = {}
            for key, remaining_key in remaining_data:
                sub_dict[remaining_key] = dictionary[key]
            if first_key not in unflattened:
                unflattened[first_key] = {}
            unflattened[first_key].update(unflatten_dict(sub_dict, sep=sep))

    return unflattened


def convert_int_keys_to_list(data: dict) -> dict | list:
    all_indices = all([is_key_index(key) for key in data.keys()])
    if not all_indices:
        for key, value in data.items():
            if isinstance(value, dict):
                data[key] = convert_int_keys_to_list(value)
        return data
    max_index = max([int(key) for key in data.keys()])

    converted: list[Any] = [None] * (max_index + 1)
    for key, value in data.items():
        if isinstance(value, dict):
            value = convert_int_keys_to_list(value)
        converted[int(key)] = value
    return converted


def convert_values_to_basic_types(data: dict) -> dict:
    converted = {}
    for key, value in data.items():
        parsed: int | float | None
        if isinstance(value, dict):
            value = convert_values_to_basic_types(value)
        elif isinstance(value, list):
            value = [convert_values_to_basic_types(item) for item in value]
        elif (parsed := try_int(value)) is not None:
            value = parsed
        elif (parsed := try_float(value)) is not None:
            value = parsed
        converted[key] = value
    return converted


def diagnostics_to_dict(status: DiagnosticStatus) -> dict:
    dictionary = {}
    for value in status.values:
        dictionary[value.key] = value.value
    unflattened = unflatten_dict(convert_values_to_basic_types(dictionary))
    list_converted = {}
    for key, value in unflattened.items():
        if isinstance(value, dict):
            value = convert_int_keys_to_list(value)
        list_converted[key] = value
    return list_converted
