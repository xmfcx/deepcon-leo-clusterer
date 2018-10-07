deepcon-leo-clusterer

Link to the slide at conference (210MB): https://drive.google.com/open?id=1MstnXP4Jf-hl8-M1xZNRFOmkYmgQTmIw

Dependencies:
* ROS Kinetic
* PCL 1.8

How to use:
I will assume you are new to ROS.


```
mkdir clusterer_ws
cd clusterer_ws
mkdir src
cd src
git clone https://github.com/xmfcx/deepcon-leo-clusterer.git
cd ..
catkin_make
```
This should build the clusterer node in your workspace.

Then you can run ```roscore``` in a terminal. And in another terminal you can run:
```
cd clusterer_ws
source devel/setup.bash
rosrun clusterer clusterer_exec
```
to run the clusterer executable.

To visualize it, in another terminal run ```rviz``` and as a configuration file choose clusterer_ws/src/deepcon-leo-clusterer/blob/master/rviz_stuff/deepcon.rviz

To feed clusterer with a point cloud stream, you can download a bag file from https://github.com/udacity/self-driving-car/tree/master/datasets

and do:
```
rosbag play filename.bag
```
to play it.
