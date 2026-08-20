#!/usr/bin/env python3
"""Normalize workstation_setup's ansible/tasks/*.yaml `vars:` blocks.

As of this writing those files declare `vars:` as a list of single-key dicts
(`- home: ...`), which current ansible-core rejects outright ("Vars in a Play
must be specified as a dictionary"). This is an upstream incompatibility in
https://github.com/aalbaali/workstation_setup, not a container issue -- this
script merges the list into a single dict in place so the unmodified upstream
playbooks can still run. Safe to delete once the upstream YAML is fixed.
"""

import glob
import sys

import yaml

for path in glob.glob(sys.argv[1] if len(sys.argv) > 1 else "*.yaml"):
    with open(path) as fh:
        docs = list(yaml.safe_load_all(fh))

    changed = False
    for doc in docs or []:
        for play in doc if isinstance(doc, list) else []:
            v = play.get("vars") if isinstance(play, dict) else None
            if isinstance(v, list):
                merged = {}
                for item in v:
                    merged.update(item)
                play["vars"] = merged
                changed = True

    if changed:
        with open(path, "w") as fh:
            yaml.safe_dump_all(docs, fh, sort_keys=False)
