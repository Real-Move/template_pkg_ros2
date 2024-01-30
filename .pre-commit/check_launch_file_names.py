#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2024 Luca Fortini
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import re
import subprocess
import sys


def check_launch_file_names(filenames):
    # Define a pattern to match filenames ending with '.launch.'
    pattern = re.compile(r'\.launch.*$')

    # Define valid file extensions for launch files
    valid_extensions = ['.py', '.xml', '.yaml']

    # Filter filenames that do not match the pattern or have invalid extensions
    failed_files = [
        filename for filename in filenames
        if not pattern.search(filename) or os.path.splitext(filename)[1] not in valid_extensions
    ]

    return failed_files

def main():
    # Get the list of staged files using git diff
    staged_files = subprocess.check_output(['git', 'diff', '--cached', '--name-only']).decode('utf-8').splitlines()

    # Extract only the filenames without paths from staged files
    staged_filenames = [os.path.basename(filename) for filename in staged_files]

    # Get the path of the launch folder relative to the script
    script_path = os.path.abspath(__file__)
    launch_folder = os.path.join(os.path.dirname(os.path.dirname(script_path)), 'launch/')

    # Get the list of filenames in the launch folder
    launch_filenames = [filename for filename in os.listdir(launch_folder) if os.path.isfile(os.path.join(launch_folder, filename))]

    # Find the intersection of staged filenames and launch filenames
    staged_files_in_launch = list(set(staged_filenames) & set(launch_filenames))

    # Check launch file names and extensions
    failed_files = check_launch_file_names(staged_files_in_launch)

    if failed_files:
        print("ERROR: Launch file names must end with '.launch.' and have valid extensions (.py, .xml, or .yaml)")
        print("Failing files:")
        for filename in failed_files:
            print(f"- {filename}")
        sys.exit(1)
    else:
        sys.exit(0)

if __name__ == '__main__':
    main()
