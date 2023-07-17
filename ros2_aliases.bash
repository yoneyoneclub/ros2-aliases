#!/bin/bash

# MIT License

# Copyright (c) 2023 Shunsuke Kimura

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

export ROS_DISTRO=humble

if [ $# != 1 ]; then
    echo \[ros2_aliases.bash\] Give a single path as an argument.
    read -p "Hit enter, and exit"
    exit 1
fi
export ROS2WS=$1
echo "Current ROS2WS : $1"

source "`dirname $BASH_SOURCE[0]`/ros2_utils.bash"

source /opt/ros/$ROS_DISTRO/setup.bash
WS_SETUP_FILE=$ROS2WS/install/setup.bash
if [ -e $WS_SETUP_FILE ]; then
  source $WS_SETUP_FILE
fi

# help
function blue { echo -e "\033[34m$1\033[m"; }
function cyan { echo -e "\033[36m$1\033[m"; }
function rahelp { # ros2-aliases-help
  echo "`cyan chws\ PATH_TO_WORKSPACE` : change ROS 2 workspace"
  echo "`blue ---colcon\ build---`"
  echo "`cyan cb`    : colcon build"
  echo "`cyan cbp`   : colcon build with packages select"
  echo "`cyan cbc`   : colcon build with cache clean"
  echo "`blue ---roscd---`"
  echo "`cyan roscd` : cd to the selected package"
  echo "`blue ---ROS\ CLI---`"
  echo "`cyan rnlist` : ros2 node list"
  echo "`cyan rninfo` : ros2 node info"
  echo "`cyan rtlist` : ros2 topic list"
  echo "`cyan rtinfo` : ros2 topic info"
  echo "`cyan rtecho` : ros2 topic echo"
  echo "`cyan rplist` : ros2 param list"
  echo "`cyan rpget`  : ros2 param get"
  echo "`cyan rpset`  : ros2 param set"
  echo "`blue ---TF---`"
  echo "`cyan view_frames\ \(namespace\)` : ros2 run tf2_tools view_frames"
  echo "`cyan tf_echo\ \[source_frame\]\ \[target_frame\]\ \(namespace\)` : ros2 run tf2_ros tf2_echo"
  echo "`blue ---rosdep---`"
  echo "`cyan rosdep_install` : rosdep install"
}

# change ROS 2 workspace
function chws {
  source $BASH_SOURCE $1
}

# colcon build
function find_pkg_dir {
  PKG=$(find $ROS2WS/src -name "package.xml" -printf "%h\n" | awk -F/ '{print $NF}' | fzf)
  find $ROS2WS/src -name $PKG | awk '{print length() ,$0}' | sort -n | awk '{ print  $2 }' | head -n 1
}

function colcon_build_packages_select {
  CMD="cd $ROS2WS && colcon build --merge-install --symlink-install --packages-select `find_pkg_dir`"
  $CMD
  source ./install/setup.bash
  history -s $CMD
}

alias cb="cd $ROS2WS && colcon build --merge-install --symlink-install && source ./install/setup.bash"
alias cbp=colcon_build_packages_select
alias cbc="cd $ROS2WS && colcon build --merge-install --symlink-install --cmake-clean-cache && source ./install/setup.bash"

# roscd
function roscd {
  CMD="cd `find_pkg_dir`"
  $CMD
  history -s $CMD
}

# rosdep
alias rosdep_install="cd $ROS2WS && rosdep install --from-paths src --ignore-src -y"
