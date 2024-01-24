# ros2_template_pkg

Run the following script inside your repo. This way all the occurences of the `ros2_template_pkg` string will be substituted by the name of your actual package.

``` bash
./change_name.py
```

Now you can build it running the following command:

```bash
colcon build --packages-select ros2_template_pkg --symlink-install
source install/setup.bash
```

Have fun developing your ROS 2 Package.

```
pip install pre-commit
pre-commit install --install-hooks
pre-commit run --all-files
```
