# ros2_template_pkg

Run the following script inside your repo. This way all the occurences of the `ros2_template_pkg` string will be substituted by the name of your actual package.

``` bash
./change_name.py
```

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
colcon build --packages-select ros2_template_pkg --symlink-install
source install/setup.bash
```

## Run the example
```bash
ros2 launch ros2_template.launch.py
```

## Run the tests
```bash
colcon test --packages-select ros2_template_pkg --ctest-args tests
colcon test-result --all --verbose
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


Have fun developing your ROS 2 Package!
