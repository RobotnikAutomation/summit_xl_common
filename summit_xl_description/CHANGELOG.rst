^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package summit_xl_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2018-05-15)
------------------
* launch modified to work
* merging with kinetic-multirobot-devel
* updating mantainers
* vrep model and scene updated
* summit_xl_description: added new version
* [summit_xl_description] odom topic changed to robotnik_base_control/odom
* robots: added 170720 versions
* [summit_xl_description]:launch files modified to work with new robot models
* [summit_xl_description]:deleted obsolete launch files
* description: new version 170421A
* description: version 170725A
* description: adding topic_prefix in xls urdf
* description: increasing mass of XL Steel
* description: using gazebo_planar_move plugin for the omnidrive configuration
* description: adding prefix_topic for astra camera and 2d laser
* description: add param scan_prefix to front_laser in summit_xl
* description: adjusting z value of ptz camera
* summit_xl_description: adding 170606A models
* description: adding robot 170123A
* description: updating steel model to the new configuration in Kinetic
* added 170518A versiom of robot_description
* description: new version 170421A - ptz
* description: new default robots/urdfs
* description: state_robot.launch param robot_model with new format (whole name/path starting from robots folder)
* description: setting name of the xacro without prefix 'multi'
* description: xacro for multi robot renamed as standard one
* description: removing xacros not migrated to multi robot config
* summit_xl_description:adding OUR arm support
* [summit_xl_description]:prefix added to structure hokuyo urdf
* default prefix updated
* [summit_xl_description]:prefix added to launch file
* [summit_xl_description]:client model added
* xacro updated to multirobot
* Merge branch 'kinetic-devel' into kinetic-multirobot-devel
* Merge branch 'indigo-devel-rc' into kinetic-devel
* Merge branch 'indigo-devel' into indigo-devel-rc
* added summit_xl_multi.urdf.xacro
* Merge branch 'indigo-devel-rc' into kinetic-devel
* summit_xl_description: added robot_model environment variable
* yaml,launch and xacro modified to multirobot
* description: HLS omni modifications
* updated robot models
* Modifying models to work with several robots
* removed camera from urdf model
* added torque constant, probably not in use
* set namespace of the ros_control plugin
* summit_xl_description: updated urdf parameters for simulation
* 1.0.10

1.1.2 (2016-09-01)
------------------
* fixed merge conflicts
* Changed xacro.py to xacro, added --inorder option and modified xmlns:xacro
* summit_xl_description: updated xacro to match jade tag syntax
* summit_xl_description: changed gazebo imu plugin to hector
* 1.0.9
* updated changelog
* Contributors: Marc Bosch-Jorge, carlos3dx

1.1.1 (2016-08-24)
------------------

1.1.0 (2016-08-24)
------------------

1.0.9 (2016-08-24)
------------------
* description: added orbbec camera to the frontal robot area
* Contributors: RomanRobotnik

1.0.8 (2016-07-12)
------------------

1.0.7 (2016-07-12)
------------------
* updated changelog
* Contributors: carlos3dx

1.0.6 (2016-07-12)
------------------

1.0.5 (2016-07-05)
------------------
* Added directory to CMakeLists
* Contributors: carlos3dx

1.0.4 (2016-06-30)
------------------
* added dependency
* Contributors: carlos3dx

1.0.3 (2016-06-29)
------------------

1.0.2 (2016-06-28)
------------------

1.0.1 (2016-06-28)
------------------
* indigo-1.0.0
* added plugin option ros_force_based_move from hector_plugins
* added urdf and meshes of extended arms (ext) version
* summit_xl_description: commenting last changes to prevent errors if robotnik_sensors is not updated
* summit_xl_description: adding new params to hokuyo sensors in robots
* added summit_xl_hls files
* changed omni wheel diam, omni plugin topic, and skid plugin distance param
* Merge branch 'indigo-devel' of https://github.com/RobotnikAutomation/summit_xl_common into indigo-devel
  Conflicts:
  summit_xl_localization/launch/navsat_transform_node.launch
* summit_xl_description: adding dependency to robotnik_sensors
* added structures folder
* added structures folder
* Update package.xml
* Pad bug solved. Now it commands 0.0 when DeadM button is released. Other minor changes
* Now summit_xl_pad only publishes when deadman's buttos is pressed.
* controlPeriod to 0.001
* deleted summit_xl_nocam.urdf.xacro
* configured nocam model and added related launch
* Removed old rubber wheel STL's.
* New omni wheels.
* Transmission modified with new style.
* New summit_xl_description and summit_xl_pad. HL version still not added
* Contributors: Dani Carbonell, Jorge Arino, mcantero, rguzman

* added plugin option ros_force_based_move from hector_plugins
* added urdf and meshes of extended arms (ext) version
* summit_xl_description: commenting last changes to prevent errors if robotnik_sensors is not updated
* summit_xl_description: adding new params to hokuyo sensors in robots
* added summit_xl_hls files
* changed omni wheel diam, omni plugin topic, and skid plugin distance param
* Merge branch 'indigo-devel' of https://github.com/RobotnikAutomation/summit_xl_common into indigo-devel
  Conflicts:
  summit_xl_localization/launch/navsat_transform_node.launch
* summit_xl_description: adding dependency to robotnik_sensors
* added structures folder
* added structures folder
* Update package.xml
* Pad bug solved. Now it commands 0.0 when DeadM button is released. Other minor changes
* Now summit_xl_pad only publishes when deadman's buttos is pressed.
* controlPeriod to 0.001
* deleted summit_xl_nocam.urdf.xacro
* configured nocam model and added related launch
* Removed old rubber wheel STL's.
* New omni wheels.
* Transmission modified with new style.
* New summit_xl_description and summit_xl_pad. HL version still not added
* Contributors: Dani Carbonell, Jorge Arino, rguzman
