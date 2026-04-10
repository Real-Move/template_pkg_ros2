ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		build-essential \
		ssh \
		software-properties-common \
		apt-transport-https \
		ca-certificates \
		lsb-release \
		pkg-config \
		gnupg \
		git \
		wget \
		curl \
		zip \
		unzip && \
	rm -rf /var/lib/apt/lists/* && \
	apt-get clean

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-dev \
        python3-pip \
        ros-${ROS_DISTRO}-ament-cmake-python \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-ament-cmake-gtest \
        ros-${ROS_DISTRO}-ament-cmake-pytest \
        ros-${ROS_DISTRO}-builtin-interfaces \
        ros-${ROS_DISTRO}-launch \
        ros-${ROS_DISTRO}-launch-ros \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ros2launch && \
	rm -rf /var/lib/apt/lists/* && \
	apt-get clean

CMD ["bash"]
