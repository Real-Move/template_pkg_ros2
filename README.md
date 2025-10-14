# template_pkg_ros2

A minimal ROS 2 package template with Docker, VS Code Devcontainer, pre-commit hooks, unit tests, and a ready-to-run launch file.

After creating your repo from this template, rename the package using the script below. This way all the occurences of the `template_pkg_ros2` string will be substituted by the name of your actual package.

``` bash
./change_name.py
```

> [!IMPORTANT]
> This probably works only if you create a repo from this template using github web interface.

## Docker

Build the Docker Image
```bash
cd_ws
docker build -t <image_name> -f .docker/Dockerfile .
```

Run the Container
```
docker run -it --rm --privileged --name <container_name> --mount type=bind,source=/ros/ros2_ws,target=/ros/ros2_ws -v /dev*:/dev* -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix <image_name> /bin/bash
```

Now you can build the ROS 2 package running the following command:

```bash
colcon build --packages-select template_pkg_ros2 --symlink-install
source install/setup.bash
```

## Run the example
```bash
ros2 launch template_pkg_ros2 ros2_template.launch.py
```

## Run the tests
```bash
colcon test --packages-select template_pkg_ros2 --ctest-args tests
colcon test-result --all --verbose

```

alternatively to run the pytests the following output is cleaner.


```
python3 -m pytest -v
```

## Pre-Commit

To use the pre-commits features you need to install `pre-commit` using pip.
```bash
pip install pre-commit
```

Then browse you package base folder and run the following commands.
```bash
pre-commit install --install-hooks
pre-commit run --all-files
```

## VS-Code Devcontainer
I added a devcontainer minimal structure to activate Python and ROS linting in VS code.

Have fun developing your ROS 2 Package!
