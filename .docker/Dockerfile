ARG ROS_DISTRO=foxy

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
        ros-${ROS_DISTRO}-ament-cmake-auto \
    	python3-colcon-common-extensions \
		python3-rosdep && \
	rm -rf /var/lib/apt/lists/* && \
	apt-get clean

WORKDIR /root/ros_ws

COPY .docker/ros_entrypoint.sh /

RUN echo 'alias cd_ws="cd /root/ros_ws"' >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
