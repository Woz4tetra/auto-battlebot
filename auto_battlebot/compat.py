"""Standard-library shims for the Python 3.10 floor."""

import sys

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

__all__ = ["tomllib"]
