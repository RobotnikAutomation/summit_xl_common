
Up to gazebo1.3 (old plugins)
=============================
Testing outdoor environment (includes gps)

Install tu-darmstadt-ros-pkg
roslaunch hector_gazebo_worlds rolling_landscape_120m.launch 
roslaunch summit_xl_description summit_xl_fotonic.launch


Testing indoor environment 

roslaunch summit_xl_description robotnik.launch
roslaunch summit_xl_description summit_xl.launch
(to control axis needs summit_xl_ctrl - plugin)


After gazebo1.5 tested gazebo-1.8.1 gazebo-1.8.3
================================================
roscore
gazebo summit_xl.world

(to control axis needs summit_xl_plugin - plugin)
