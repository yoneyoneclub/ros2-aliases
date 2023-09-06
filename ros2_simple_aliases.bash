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

export ROS2_ALIASES=$BASH_SOURCE
export ROS_DISTRO=humble

function red  { echo -e "\033[31m$1\033[m"; }
function blue { echo -e "\033[34m$1\033[m"; }
function cyan { echo -e "\033[36m$1\033[m"; }

# $# arg num
# arg = 0 
# if [ $# = 0 ]; then #
#   red "[ros2 aliases] Give at least one path as an argument."
#   red "[Usage 1] source PATH_TO_CLONE/ros2_aliases.bash ROS_WORKSPACE"
#   red "[Usage 2] source PATH_TO_CLONE/ros2_aliases.bash ROS_WORKSPACE COLCON_BUILD_CMD"
#   red "[Usage 3] source PATH_TO_CLONE/ros2_aliases.bash CONFIG_FILE"
#   return
# fi

# source other scripts
source "`dirname $ROS2_ALIASES`/ros2_utils.bash"

# ros2 aliases help
function rahelp {
  blue "---ROS\ CLI---"
  echo "`cyan rnlist` : ros2 node list"
  echo "`cyan rninfo` : ros2 node info"
  echo "`cyan rtlist` : ros2 topic list"
  echo "`cyan rtinfo` : ros2 topic info"
  echo "`cyan rtecho` : ros2 topic echo"
  echo "`cyan rplist` : ros2 param list"
  echo "`cyan rpget`  : ros2 param get"
  echo "`cyan rpset`  : ros2 param set"
  blue "---TF---"
  echo "`cyan view_frames\ \(namespace\)` : ros2 run tf2_tools view_frames"
  echo "`cyan tf_echo\ \[source_frame\]\ \[target_frame\]\ \(namespace\)` : ros2 run tf2_ros tf2_echo"
  blue "---rosdep---"
  echo "`cyan rosdep_install` : rosdep install"
  blue "---offical---"
  echo "`cyan "ros2 -h"` : The Official help"
}


