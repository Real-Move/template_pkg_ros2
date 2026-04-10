# template_pkg_ros2

[![ROS CI Test Build](https://github.com/Real-Move/template_pkg_ros2/actions/workflows/ros-ci.yml/badge.svg)](https://github.com/Real-Move/template_pkg_ros2/actions/workflows/ros-ci.yml)

`template_pkg_ros2` is a minimal ROS 2 package template for starting new nodes and small applications. It includes C++ and Python executables, example launch files, unit tests, Docker support, and pre-commit integration.

## Overview

Use this repository as a starting point for a new ROS 2 package when you want:

- A mixed C++ and Python package layout
- Ready-to-run example executables
- Example launch files, including an included launch file
- Basic smoke tests for C++ and Python code
- A Docker-based development workflow
- Pre-commit hooks for basic code quality checks

## Rename the Package

After creating a repository from this template, rename the package before starting development:

```bash
./change_name.py
```

This script replaces occurrences of `template_pkg_ros2` with your target package name.

> [!IMPORTANT]
> The renaming script is intended for repositories created directly from this template and may not cover every custom setup.


## Package Contents

### Features

- C++ ROS 2 nodes built with `ament_cmake`
- Python executables installed with the package
- Launch files for starting multiple nodes together
- Example tests for both `gtest` and `pytest`
- Docker and devcontainer support for local development

### Installed Executables

The package installs the following executables:

- `minimal_cpp_node`
- `hello_world_node`
- `logging_demo_node`
- `minimal_py_node.py`
- `clock_node.py`
- `hello_world_node.py`

### Launch Files

The package installs the following launch files:

- `ros2_template.launch.py`
- `example_include.launch.py`

`ros2_template.launch.py` starts:

- `minimal_cpp_node`
- `minimal_py_node.py`
- `logging_demo_node`
- `example_include.launch.py`


## Build

From the workspace root:

```bash
colcon build --packages-select template_pkg_ros2 --symlink-install
source install/setup.bash
```

## Run

Launch the main example:

```bash
ros2 launch template_pkg_ros2 ros2_template.launch.py
```

Run individual executables:

```bash
ros2 run template_pkg_ros2 minimal_cpp_node
ros2 run template_pkg_ros2 logging_demo_node
ros2 run template_pkg_ros2 hello_world_node
ros2 run template_pkg_ros2 minimal_py_node.py
ros2 run template_pkg_ros2 clock_node.py
ros2 run template_pkg_ros2 hello_world_node.py
```

`clock_node.py` publishes the current ROS time on `clock_topic` once per second. `hello_world_node.py` logs once and exits.

## Docker

Build the Docker image from the workspace root:

```bash
docker build -t <image_name> -f .docker/Dockerfile .
```

Run the container:

```bash
docker run -it --rm \
  --privileged \
  --name <container_name> \
  --mount type=bind,source=/ros/ros2_ws,target=/ros/ros2_ws \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  <image_name> \
  /bin/bash
```

Then build the package inside the container:

```bash
colcon build --packages-select template_pkg_ros2 --symlink-install
source install/setup.bash
```
## Test

Run tests for this package:

```bash
colcon test --packages-select template_pkg_ros2
colcon test-result --verbose
```

## Pre-commit

Install `pre-commit`:

```bash
pip install pre-commit
```

Enable the hooks and run them across the repository:

```bash
pre-commit install --install-hooks
pre-commit run --all-files
```
