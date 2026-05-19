#!/usr/bin/env python3
# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

INCLUDE_RE = re.compile(r'#include\s*(?:"([^"]+)"|<([^>]+)>)')


def discover_local_prefixes() -> set[str]:
    """Infer local include prefixes from repository metadata.

    Priority:
    1. ROS package name from package.xml in repo root.
    2. Repository folder name as fallback.
    """
    prefixes: set[str] = set()
    repo_root = Path(__file__).resolve().parent.parent

    package_xml = repo_root / "package.xml"
    if package_xml.exists():
        try:
            root = ET.fromstring(package_xml.read_text())
            name_elem = root.find("name")
            if name_elem is not None and name_elem.text:
                prefixes.add(name_elem.text.strip())
        except ET.ParseError:
            # Keep hook resilient: fallback to repository name.
            pass

    prefixes.add(repo_root.name)
    return {p for p in prefixes if p}


LOCAL_PREFIXES = discover_local_prefixes()


def fix(text: str) -> str:
    def repl(match):
        path = match.group(1) or match.group(2)

        # if it starts with one of your local prefixes → keep ""
        if any(path == p or path.startswith(f"{p}/") for p in LOCAL_PREFIXES):
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
