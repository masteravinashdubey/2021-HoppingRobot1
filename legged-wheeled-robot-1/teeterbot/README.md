## Legged wheeled robot

This package is consists of a Legged wheeled which was developed and tested on Gazebo using ROS Melodic. A mathematical model of the bot was developed using state space modelling, following which the model was fed into a Linear Quadratic Regulator to obtain robust control.

#### Package Structure 

This package consists of two main ROS packages:-

- teeterbot_description: contains the bot's URDF with rviz visualizing launch file.

- teeterbot_gazebo: contains the bot low level torque controllers, gazebo launch files, and all the main scripts (ie. LQR, PID, pdfs to help understand the mathematical model of the bot, and an octave file to derive A and B matrices of state equation) .

#### Launching Balance Bot  

To launch legged wheeled robot on Gazebo with LQR:-

```
roslaunch teeterbot_gazebo lwr_empty_world_control.launch 

```


To view balance bot on Rviz:-

```
roslaunch teeterbot_description teeterbot_rviz.launch 
```

