# AGENTS.md

This repository contains one or more ROS 2 packages.

Rules:
- Make minimal changes.
- Work only in the relevant package unless explicitly told otherwise.
- When updating `.docker/Dockerfile`, only add what is required for the selected package.
- Do not change unrelated packages, launch files, topics, or message definitions.
- Before editing, briefly state which package you think is the target.
- After editing, report what changed and why.
