#!/bin/bash

function ros_source_env()
{
	if [ -f "$1" ]; then
		echo "sourcing   $1"
		source "$1"
	else
		echo "notfound   $1"
	fi
}

ros_source_env "$ROS_ROOT/setup.bash"

exec "$@"
