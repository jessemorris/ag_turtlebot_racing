# Augmented Reality Turtlebot Racing (AMME4710 @ Usyd)

## Dependancies ##

- ROS Melodic
- Turtlebot
- Catkin Build tools
```
sudo apt-get install python-catkin-tools
```

## ROS Networking Setup ##

### On the Turtlebot ###
In the __.bashrc__ add:

```
export ROS_MASTER_URI=http://<IP of computer>:11311
export ROS_HOSTNAME=<IP of raspberry pi>
```

Source the __.bashrc__


### On the master computer ###


```
export ROS_MASTER_URI=http://<IP of computer>:11311
export ROS_HOSTNAME=<IP of computer>
```

Source the __.bashrc__


### Build and Install ###

Using Catkin Build tools build the workspace using
```
catkin build
```

See https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html for the catkin build cheatsheat if you've never used it before


## Capabilities ##

Augmented reality turtlebot racing (inspired by Mario Cart AR) for AMME4710 at Usyd
