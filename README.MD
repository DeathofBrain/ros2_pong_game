# ros2_ball_pong_game

## Description
This is a simple ROS2 package that implements a ball pong game. This is a assignment for the Upper New Teammates of Mosasaurus. And this code is based on this repository:[ros-turtle-pong](https://github.com/fjp/ros-turtle-pong/) and was adapted to work with ROS2. 

## Installation
To install the package, clone the repository in your ROS2 workspace and build it using colcon:
```
git clone https://github.com/DeathofBrain/ros2_pong_game.git
cd ~/ros2_pong_game/src
colcon build
```

## Usage
To run the game, source the ROS2 workspace and run the node executable:
```
source ~/ros2_pong_game/install/setup.bash
ros2 run turtlesim turtlesim_node
ros2 run ball_pong ball_pong
ros2 run teleop_key teleop_key
```

## Controls
The game is controlled using the keyboard. The controls are as follows:
- Player 1: W and S keys to move the paddle up and down, respectively.
- Player 2: Up and Down arrow keys to move the paddle up and down, respectively.
