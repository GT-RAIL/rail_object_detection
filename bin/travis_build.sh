#!/usr/bin/env bash
# Build from travis translated into a shell script
set -ex

# Add a ROS user if this is docker and you logged in as root
if [ "$(whoami)" == "root" ]
then
    TEST_DIR="/test"
    mkdir $TEST_DIR
    cp -r * $TEST_DIR
    adduser --disabled-password --gecos "ROS,,," --quiet ros
    usermod -aG sudo ros
    chown -R ros:ros $TEST_DIR
    cd $TEST_DIR
    echo "ros ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
    sudo -u ros -H -s $TEST_DIR/bin/travis_build.sh
    exit
fi

# Environment variables
ROS_DISTRO=indigo
ROS_CI_DESKTOP="`lsb_release -cs`"  # e.g. [precise|trusty|...]
CI_SOURCE_PATH=$(pwd)
ROSINSTALL_FILE=$CI_SOURCE_PATH/dependencies.rosinstall
CATKIN_OPTIONS=$CI_SOURCE_PATH/catkin.options
ROS_PARALLEL_JOBS='-j8 -l6'

# Install ROS
sudo apt-get update -qq
sudo apt-get install -y wget
sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $ROS_CI_DESKTOP main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin build-essential cmake pkg-config
source /opt/ros/$ROS_DISTRO/setup.bash

# Prepare rosdep to install dependencies.
sudo rosdep init
rosdep update
sudo apt-get install -y libeigen3-dev

# Create and install the catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# Create the devel/setup.bash (run catkin_make with an empty workspace) and
# source it to set the path variables.
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Add the package under integration to the workspace using a symlink.
cd ~/catkin_ws/src
ln -s $CI_SOURCE_PATH .

# Install dependencies
cd ~/catkin_ws/src
wstool init
if [[ -f $ROSINSTALL_FILE ]] ; then wstool merge $ROSINSTALL_FILE ; fi
wstool up
cd ~/catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Build and test
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws
catkin_make $( [ -f $CATKIN_OPTIONS ] && cat $CATKIN_OPTIONS )
catkin_make install

# Run the tests, ensuring the path is set correctly.
source devel/setup.bash
cd $CI_SOURCE_PATH
echo "BUILD SUCCESS"
