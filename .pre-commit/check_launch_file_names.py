#!/usr/bin/env python3
# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import os
import sys

VALID_SUFFIXES = (".launch.py", ".launch.xml", ".launch.yaml")


def check_launch_file_names(filenames):
    failed_files = []
    for filename in filenames:
        basename = os.path.basename(filename)
        if not basename.endswith(VALID_SUFFIXES):
            failed_files.append(filename)
    return failed_files


def main():
    filenames = sys.argv[1:]

    # Only check files inside a launch directory.
    launch_files = [
        filename
        for filename in filenames
        if "/launch/" in filename or filename.startswith("launch/")
    ]

    if not launch_files:
        sys.exit(0)

    failed_files = check_launch_file_names(launch_files)

    if failed_files:
        print(
            "ERROR: Launch file names must end with '.launch.' and have valid "
            "extensions (.py, .xml, or .yaml)"
        )
        print("Failing files:")
        for filename in failed_files:
            print(f"- {filename}")
        sys.exit(1)

    sys.exit(0)


if __name__ == "__main__":
    main()
