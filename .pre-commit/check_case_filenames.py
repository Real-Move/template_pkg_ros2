#!/usr/bin/env python3
# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import re
import sys
from pathlib import Path

PASCAL_CASE = re.compile(r"^[A-Z][a-zA-Z0-9]*$")


def is_pascal_case(name):
    return bool(PASCAL_CASE.match(name))


def pascal_to_snake(name):
    """Convert PascalCase -> snake_case"""
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    s2 = re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1)
    return s2.lower()


def main():
    bad_files = []

    for filename in sys.argv[1:]:
        path = Path(filename)
        stem = path.stem

        if is_pascal_case(stem):
            suggested = pascal_to_snake(stem) + path.suffix
            bad_files.append((filename, suggested))

    if bad_files:
        print("ERROR: PascalCase filenames detected. Use snake_case instead.\n")
        for original, suggested in bad_files:
            print(f"  {original}")
            print(f"    → suggested: {suggested}")
        print("\nRename the files before committing.")
        sys.exit(1)

    sys.exit(0)


if __name__ == "__main__":
    main()
