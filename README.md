# ros2-aliases
Collection of functions and aliases for ROS2 development

![](https://github.com/tonynajjar/ros2-aliases/blob/main/usage.gif)

# Prerequisites

- [fzf](https://github.com/junegunn/fzf#installation)
  For Ubuntu simply: 
  ```
  sudo apt install fzf
  ```
  For more install options refer to the documentation

- Bash or Zsh

# Installation

- Clone the repo: `git clone https://github.com/kimushun1101/ros2-aliases.git PATH_TO_CLONE`  
  For example:
  ```
  git clone https://github.com/kimushun1101/ros2-aliases.git ~/ros2-aliases
  ```

Bash:
- Add ros2_aliases.bash to your bashrc with PATH_TO_YOUR_ROS2_WS: `echo 'source PATH_TO_CLONE/ros2_aliases.bash PATH_TO_YOUR_ROS2_WS' >> ~/.bashrc`  
  For example:
  ```
  echo 'source ~/ros2-aliases/ros2_aliases.bash ~/ros2_ws' >> ~/.bashrc
  ```

Zsh:
- help wanted

# Usage

`rahelp` shows `ros2_aliases help`.

## Topics

| Command | Alias |
| --- | --- |
| `ros2 topic list` | `rtlist` |
| `ros2 topic echo` | `rtecho`|
| `ros2 topic info` | `rtinfo`|

## Nodes

| Command | Alias |
| --- | --- |
| `ros2 node list` | `rnlist` |
| `ros2 node info` | `rninfo`|

## Services

| Command | Alias |
| --- | --- |
| `ros2 service list` | `rslist` |

## Parameters

| Command | Alias |
| --- | --- |
| `ros2 param list` | `rplist` |
| `ros2 param get`  | `rpget`|
| `ros2 param set`  | `rpset`|

## TF

| Command | Alias | Arguments |
| --- | --- | --- |
| `ros2 run tf2_tools view_frames` | `view_frames` | namespace of TF topic [Optional] |
| `ros2 run tf2_ros tf2_echo` | `tf2_echo`| source_frame [Required], target_frame [Required], namespace of TF topic [Optional] |

## Colcon

| Command | Alias |
| --- | --- |
| `cd $ROS2WS` && `colcon build --merge-install --symlink-install` | `cb` |
| `cd $ROS2WS` && `colcon build --merge-install --symlink-install --packages-select` | `cbp`|
| `cd $ROS2WS` && `colcon build --merge-install --symlink-install --cmake-clean-cache ` | `cbc`|

## Rosdep

| Command | Alias |
| --- | --- |
| `cd $ROS2WS` && `rosdep install --from-paths src --ignore-src -y` | `rosdep_install` |