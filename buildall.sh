#!/bin/bash

set -e

ROS2_WS=ros2_ws
NAV2_DEPS=navstack_dependencies_ws
TB_WS=turtlebot3_ws
NAV2_WS=navigation2_ws
BINARY_WS=/tmp/ros2-linux
ROS2_BINARY_URL=https://ci.ros2.org/view/packaging/job/packaging_linux/lastSuccessfulBuild/artifact/ws/ros2-package-linux-x86_64.tar.bz2

ROS2_BUILD_TYPE=Release
NAV2_DEPS_BUILD_TYPE=Release
TB_BUILD_TYPE=Release
NAV2_BUILD_TYPE=Debug

BINARY=False

clean_workspace() {
  local CWD=`pwd`
  cd $1
  echo cleaning $1
  rm -rf build install log
  cd $CWD
}

build_workspace() {
  local CWD=`pwd`
  cd $1
  echo building $1
  colcon build --symlink-install --merge-install --packages-skip ros1_bridge --cmake-args -DCMAKE_BUILD_TYPE=$2
  cd $CWD
}

source_workspace() {
  source $1/install/setup.bash
}

update_ros2_ws() {
  local CWD=`pwd`
  cd $ROS2_WS
  rm -rf src
  rm -f ros2.repos
  mkdir src
  if [ "$ROS2_REPOS" == "" ]; then
    wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
    #wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
    vcs import src < ros2.repos
    vcs export src --exact > ~/ros2versions/`date +%F-%H-%M-%S`.repos
  else
    vcs import src < $ROS2_REPOS
  fi
  cd $CWD
}

update_ros2_binary() {
  local CWD=`pwd`
  rm -rf $BINARY_WS
  cd /tmp
  wget -O - $ROS2_BINARY_URL | tar jxvf -
  cd $CWD
}

update_nav2_dependencies() {
  local CWD=`pwd`
  cd $NAV2_DEPS
  rm -rf src
  mkdir src
  vcs import src < ../$NAV2_WS/navigation2/tools/ros2_dependencies.repos
  cd $CWD
}

update_tb3() {
  local CWD=`pwd`
  cd $TB_WS
  vcs pull
  cd $CWD
}

update_tb3_repos() {
  local CWD=`pwd`
  cd $TB_WS
  rm -rf src
  rm turtlebot3.repos
  mkdir src
  wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
  vi turtlebot3.repos
  vcs import src < turtlebot3.repos
  cd $CWD
}

for opt in "$@" ; do
  case "$opt" in
    clean)
      clean_workspace $ROS2_WS
      clean_workspace $NAV2_DEPS
      clean_workspace $TB_WS
      clean_workspace $NAV2_WS
      ;;
    release)
      ROS2_BUILD_TYPE=Release
      NAV2_DEPS_BUILD_TYPE=Release
      TB_BUILD_TYPE=Release
      NAV2_BUILD_TYPE=Release
      ;;
    debug)
      ROS2_BUILD_TYPE=Debug
      NAV2_DEPS_BUILD_TYPE=Debug
      TB_BUILD_TYPE=Release
      NAV2_BUILD_TYPE=Debug
      ;;
    update)
      update_ros2_ws
      exit 0
      ;;
    updatebinary)
      update_ros2_binary
      exit 0
      ;;
    updatedeps)
      update_nav2_dependencies
      exit 0
      ;;
    updatetb3)
      update_tb3
      exit 0
      ;;
    updatetb3repos)
      update_tb3_repos
      exit 0
      ;;
    binary)
      ROS2_WS=$BINARY_WS
      BINARY=True
      ;;
    *)
      echo "Invalid parameter $opt"
      exit 1
      ;;
  esac
done

if [ "$BINARY" == "True" ]; then
  source $ROS2_WS/setup.bash
else
  build_workspace $ROS2_WS $ROS2_BUILD_TYPE
  source_workspace $ROS2_WS
fi
build_workspace $NAV2_DEPS $NAV2_DEPS_BUILD_TYPE
source_workspace $NAV2_DEPS
build_workspace $NAV2_WS $NAV2_BUILD_TYPE
source_workspace $NAV2_WS
build_workspace $TB_WS $TB_BUILD_TYPE
source_workspace $TB_WS
