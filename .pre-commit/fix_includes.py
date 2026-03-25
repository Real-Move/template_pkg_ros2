#!/usr/bin/env python3
# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import re
import sys
from pathlib import Path

# Define your project prefixes
LOCAL_PREFIXES = [
    "realmove_gui_ros2",
]

INCLUDE_RE = re.compile(r'#include\s+"([^"]+)"')


def fix(text: str) -> str:
    def repl(match):
        path = match.group(1)

        # if it starts with one of your local prefixes → keep ""
        if any(path.startswith(p) for p in LOCAL_PREFIXES):
            return f'#include "{path}"'

        # otherwise → external → use <>
        return f"#include <{path}>"

    return INCLUDE_RE.sub(repl, text)


def main():
    changed = False
    for f in sys.argv[1:]:
        p = Path(f)
        original = p.read_text()
        new = fix(original)
        if new != original:
            p.write_text(new)
            changed = True
    return 1 if changed else 0


if __name__ == "__main__":
    raise SystemExit(main())
