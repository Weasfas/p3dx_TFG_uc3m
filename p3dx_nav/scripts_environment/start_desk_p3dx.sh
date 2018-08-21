#!/bin/bash
echo "Creating enviroment..."
#echo "Setting perimissions on port: /dev/ttyUSB0"

#sudo chmod 777 /dev/ttyUSB0

#Current dir
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
#Target dir
SPER="$DIR/catkin_ws"

if [ "$SPER" != "$DIR" ]; then
 cd "$SPER"
fi

echo "Setting ROS Environment variables: ROS_IP and ROS_MASTER_URI"
export ROS_IP=10.42.0.13
export ROS_MASTER_URI=http://10.42.0.1:11311
. devel/setup.bash
