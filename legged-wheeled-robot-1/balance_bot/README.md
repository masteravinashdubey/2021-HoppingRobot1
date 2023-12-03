## Balance Bot

This package is consists of a balance bot which was developed and tested on Gazebo using ROS Melodic. A mathematical model of the bot was developed using state space modelling, following which the model was fed into a Linear Quadratic Regulator to obtain robust control.

#### Package Structure 

This package consists of two main ROS packages:-

- balance_bot_description: contains the bot's URDF with rviz visualizing launch file.

- balance_bot_gazebo: contains the bot low level torque controllers, gazebo launch files, and all the main scripts (ie. LQR, PID, pdfs to help understand the mathematical model of the bot, and an octave file to derive A and B matrices of state equation) .

#### Launching Balance Bot  

To launch balance bot on Gazebo with LQR:-

```
roslaunch balance_bot_gazebo balance_bot_empty_world_lqr.launch
```



To launch balance bot on Gazebo with with PID:-

```
roslaunch balance_bot_gazebo balance_bot_empty_world_pid.launch
```



To view balance bot on Rviz:-

```
roslaunch balance_bot_description balance_bot_rviz.launch
```

