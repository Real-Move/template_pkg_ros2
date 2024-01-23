# template_pkg_ros2

Run the following script inside your repo. This way all the occurences of the `template_pkg_ros2` string will be substituted by the name of your actual package.

``` bash
./change_name.py
```

Now you can build it running the following command:

```bash
colcon build --packages-select template_pkg_ros2 --symlink-install
source install/setup.bash
```

Have fun developing your ROS 2 Package.