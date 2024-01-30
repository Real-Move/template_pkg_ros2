#!/usr/bin/env python

import os
import subprocess

TEMPLATE_NAME = "ros2_template_pkg"

def get_git_repo_name(directory):
    """
    Get the name of the Git repository in the specified directory.

    Args:
        directory (str): The directory path to check for a Git repository.

    Returns:
        str: The name of the Git repository if found, or None if the directory is not a Git repository.

    Raises:
        subprocess.CalledProcessError: If the 'git rev-parse --show-toplevel' command fails.
    """
    try:
        # Run 'git rev-parse --show-toplevel' to get the top-level directory of the Git repository
        git_top_level = subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], cwd=directory, text=True).strip()

        # Extract the last component of the path, which is the repository name
        repo_name = os.path.basename(git_top_level)
        return repo_name
    except subprocess.CalledProcessError:
        # Handle the case where the directory is not a Git repository
        return None

def replace_string(directory, old_string, new_string):
    """
    Recursively replaces occurrences of a string in file names, folder names, and file contents within a directory.

    Args:
        directory (str): The root directory where the replacement will take place.
        old_string (str): The string to be replaced.
        new_string (str): The string to replace occurrences of the old string.

    Returns:
        None
    """
    for root, dirs, files in os.walk(directory, topdown=True):

        if '.git' in dirs:
            dirs.remove('.git')

        # Replace occurrences in file names
        for filename in files:
            new_filename = filename.replace(old_string, new_string)
            os.rename(os.path.join(root, filename), os.path.join(root, new_filename))

        # Replace occurrences in folder names
        for folder_name in dirs:

            new_folder_name = folder_name.replace(old_string, new_string)

            os.rename(os.path.join(root, folder_name), os.path.join(root, new_folder_name))

        # Replace occurrences in file contents
        for filename in files:
            filepath = os.path.join(root, filename)

            with open(filepath, 'r') as file:
                file_content = file.read()
                file_content = file_content.replace(old_string, new_string)
            with open(filepath, 'w') as file:
                file.write(file_content)

if __name__ == "__main__":

    base_directory = os.path.dirname(os.path.realpath(__file__))
    repo_name = get_git_repo_name(base_directory)

    replace_string(base_directory, TEMPLATE_NAME, repo_name)

    # delete the file once finished
    os.remove(os.path.join(base_directory, 'change_name.py'))
