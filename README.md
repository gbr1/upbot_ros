# upbot_ros
## Setup
In order to run this package you need:
- [erwhi-hedgehog-ros](https://github.com/gbr1/erwhi-hedgehog-ros) package
- [sengi_ros](https://github.com/gbr1/sengi_ros) package
- [upboard_ros](https://github.com/gbr1/upboard_ros) package
- [realsense](https://github.com/intel-ros/realsense) package

## How to use
To emulate Sengi:<br>
`roslaunch upbot_ros simulation.launch`<br>
<br>
To view robot with Intel Realsense:<br>
`roslaunch upbot_ros simulation.launch realsense:=true`<br>
<br>
To try Intel Realsense and RTABmap:<br>
`roslaunch upbot_ros d435_slam.launch`<br>

<br>
<br>



***Copyright (c) 2019 Giovanni di Dio Bruno under MIT license.***