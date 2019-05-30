# upbot_ros

Check the [wiki](https://github.com/gbr1/upbot_ros/wiki) to learn more.

## How to install
Open terminal and type:<br>
`$ cd ~/catkin_ws/src`<br>
`$ git clone https://github.com/gbr1/upbot_ros.git`<br>
`$ cd ..`<br>
`$ catkin_make`<br>
`$ catkin_make install`

<br>


## How to use
To emulate Sengi:<br>
`$ roslaunch upbot_ros simulation.launch`<br>
<br>
To view robot with Intel Realsense:<br>
`$ roslaunch upbot_ros simulation.launch realsense:=true`<br>
<br>
To try Intel Realsense and RTABmap:<br>
`$ roslaunch upbot_ros d435_slam.launch`<br>

<br>
<br>

## Dependencies
In order to run this package you need:
- [erwhi-hedgehog-ros](https://github.com/gbr1/erwhi-hedgehog-ros) package
- [sengi_ros](https://github.com/gbr1/sengi_ros) package
- [upboard_ros](https://github.com/gbr1/upboard_ros) package
- [realsense](https://github.com/intel-ros/realsense) package

## More
If you are interested in emulated robot, please check this repo: [Erwhi Hedgehog](https://github.com/gbr1/erwhi-hedgehog).<br>
<br>
If you were at AWS Re:MARS 2019, you can check code used in demo here: [Erwhi Hedgehog demo at AWS Re:MARS 2019](https://github.com/gbr1/erwhi_remars_2019)

<br>
<br>



***Copyright (c) 2019 Giovanni di Dio Bruno under MIT license.***
