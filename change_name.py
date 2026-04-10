#!/usr/bin/env python3

# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import os
import re
import subprocess
import sys
from pathlib import Path

TEMPLATE_NAME = "template_pkg_ros2"
GITHUB_ORG = "Real-Move"  # change if you use a different org/user


def get_git_repo_name(directory):
    """
    Get the name of the Git repository in the specified directory.
    """
    try:
        git_top_level = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], cwd=directory, text=True
        ).strip()
        return os.path.basename(git_top_level)
    except subprocess.CalledProcessError:
        return None


def replace_string(directory, old_string, new_string):
    """
    Recursively replaces occurrences of a string in file names, folder names,
    and file contents within a directory.

    Skips .git.
    """
    for root, dirs, files in os.walk(directory, topdown=True):
        if ".git" in dirs:
            dirs.remove(".git")

        # Rename files
        for filename in files:
            if old_string in filename:
                old_path = os.path.join(root, filename)
                new_filename = filename.replace(old_string, new_string)
                new_path = os.path.join(root, new_filename)
                if old_path != new_path:
                    os.rename(old_path, new_path)

        # Rename folders
        for i, folder_name in enumerate(dirs):
            if old_string in folder_name:
                old_path = os.path.join(root, folder_name)
                new_folder_name = folder_name.replace(old_string, new_string)
                new_path = os.path.join(root, new_folder_name)
                if old_path != new_path:
                    os.rename(old_path, new_path)
                    dirs[i] = new_folder_name  # keep os.walk in sync

        # Replace contents
        for filename in files:
            filepath = os.path.join(root, filename)
            try:
                with open(filepath, "r", encoding="utf-8") as f:
                    content = f.read()
                new_content = content.replace(old_string, new_string)
                if new_content != content:
                    with open(filepath, "w", encoding="utf-8") as f:
                        f.write(new_content)
            except UnicodeDecodeError:
                continue  # skip binary/non-utf8 files


def get_ci_workflow_metadata(repo_root):
    """
    Return the CI workflow filename and display name if a likely CI workflow exists.
    """
    workflows_dir = Path(repo_root) / ".github" / "workflows"
    if not workflows_dir.exists():
        return None, None

    candidates = sorted(workflows_dir.glob("*.y*ml"))
    for workflow_path in candidates:
        try:
            content = workflow_path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue

        if "pull_request:" not in content and "push:" not in content:
            continue

        name_match = re.search(r"^name:\s*(.+?)\s*$", content, flags=re.MULTILINE)
        workflow_name = name_match.group(1).strip() if name_match else "CI"
        return workflow_path.name, workflow_name

    return None, None


def update_readme(repo_root, template_name, repo_name, github_org=GITHUB_ORG):
    """
    README.md is at repo root.
    Keep only:
      - title
      - first GitHub Actions badge line (with updated repo path)
    """
    readme_path = os.path.join(repo_root, "README.md")
    if not os.path.exists(readme_path):
        return

    with open(readme_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    # First H1 title
    title_line = None
    for line in lines:
        if line.strip().startswith("# "):
            title_line = line.strip()
            break

    # First GitHub Actions badge line
    badge_line = None
    badge_regex = re.compile(
        r"\[!\[.*?\]\(https://github\.com/.+?/actions/workflows/.+?/badge\.svg\)\]\(https://github\.com/.+?/actions/workflows/.+?\)"
    )
    for line in lines:
        if badge_regex.search(line.strip()):
            badge_line = line.strip()
            break

    # Fallbacks
    if title_line is None:
        title_line = f"# {repo_name}"
    else:
        title_line = title_line.replace(template_name, repo_name)

    if badge_line is None:
        workflow_file, workflow_name = get_ci_workflow_metadata(repo_root)
        if workflow_file is None:
            workflow_file = "ros-ci.yml"
        if workflow_name is None:
            workflow_name = "ROS CI Test Build"
        badge_line = (
            f"[![{workflow_name}](https://github.com/{github_org}/{repo_name}/actions/workflows/{workflow_file}/badge.svg)]"
            f"(https://github.com/{github_org}/{repo_name}/actions/workflows/{workflow_file})"
        )
    else:
        badge_line = badge_line.replace(template_name, repo_name)

    new_readme = title_line + "\n\n" + badge_line + "\n"

    with open(readme_path, "w", encoding="utf-8") as f:
        f.write(new_readme)


def parse_args(argv):
    dry_run = "--dry-run" in argv
    assume_yes = "--yes" in argv

    unsupported = [arg for arg in argv if arg not in {"--dry-run", "--yes"}]
    if unsupported:
        raise ValueError(
            f"Unsupported arguments: {' '.join(unsupported)}. Supported flags: --dry-run, --yes."
        )

    return dry_run, assume_yes


if __name__ == "__main__":
    dry_run, assume_yes = parse_args(sys.argv[1:])
    repo_root = os.path.dirname(os.path.realpath(__file__))
    repo_name = get_git_repo_name(repo_root)

    if repo_name is None:
        raise RuntimeError("Not inside a git repository; cannot determine repo name.")

    print(f"Template package name: {TEMPLATE_NAME}")
    print(f"Repository name: {repo_name}")
    if dry_run:
        print("Dry run selected. No files will be modified.")
        sys.exit(0)

    if not assume_yes:
        reply = input(
            "This will rename files and replace text recursively in this repository, then delete "
            "change_name.py. Continue? [y/N]: "
        ).strip()
        if reply.lower() not in {"y", "yes"}:
            print("Aborted.")
            sys.exit(1)

    replace_string(repo_root, TEMPLATE_NAME, repo_name)

    # Rewrite README at repo root
    update_readme(repo_root, TEMPLATE_NAME, repo_name)

    # delete the script once finished
    os.remove(os.path.join(repo_root, "change_name.py"))

    print("changed name successfully")
