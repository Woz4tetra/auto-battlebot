"""Standard-library shims for the Python 3.10 floor (JetPack 6's system python).

Import `tomllib` from here, never directly: it is stdlib only from 3.11, and the JetPack 6 venv
is 3.10 because apt builds TensorRT's bindings for the system python only.
"""

import sys

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

__all__ = ["tomllib"]
