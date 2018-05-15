^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package summit_xl_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2018-05-15)
------------------
* merging with kinetic-multirobot-devel
* control: updating mantainers
* Solved depreciation warnings
* [summit_xl_control] enable_odom_tf param moved before the load of the controller
* [summit_xl_control] controllers of sim and real robot
* Add missing dependencies
* control: added topic docker/cmd_vel to twist_mux
* control: enabling setting params for joint names based on robot_id
* moving amcl,map_server from nav to loc pkg. Creating arguments for most of launch files
* summit_xl_control: cleaning and parametrizing configuration files to support skid & omni configurations for XL and XL Steel
* summit_xl_control::prefix added to frames
* Merge branch 'kinetic-devel' into kinetic-multirobot-devel
* Merge branch 'indigo-devel-rc' into kinetic-devel
* summit_xl_control: corrected wheel diameter
* cmd_watchdog param to 0.2 by default (for logitech)
* added summit_xl_controller/SummitXLController ros control cfg file
* updated odom params for diff_drive_controller and ekf filter config
* changed covariance values in diff_drive_controller
* updated wheel odometry wheelbase param
* removed robot_state_publisher, check sim bringup launch
* changed e-stop topic name to correct one
* added launch to control each wheel independently
* updated summit_xl_control to use the diff_drive_controller

1.1.1 (2016-08-24)
------------------

1.0.10 (2016-08-24)
-------------------

1.0.9 (2016-07-13)
------------------

1.0.8 (2016-07-12)
------------------

1.0.7 (2016-07-12)
------------------

1.0.6 (2016-07-04)
------------------

1.0.5 (2016-07-01)
------------------
* modified CMakelists
* Contributors: carlos3dx

1.0.4 (2016-07-01)
------------------

1.0.3 (2016-07-01)
------------------

1.0.2 (2016-07-01)
------------------

1.0.1 (2016-06-28)
------------------
* modified CMakeLists.txt and added urls and maintainers to package files
* minor changes
* changed topic for twist output
* changed cmd_vel_out from /summit_xl_control/cmd_vel to /summit_xl/robot_control/command
* control: fixing twist_mux output topic
* control: fixing typo
* updated maitainer
* added control files for hls version
* Mux added. Old files removed.
* Removed old packages and new structure defined.
* fixing package dependencies
* summit_xl_description: adding axis sensor to summit X chasis
* adding scissor movement
* Not anymore multi.
* 1.0.0
* Fixed ptz controller error
* Allows multiple simulations of summit xl (not omni wheels yet)
* Now it uses a tf_prefix for the odom->base_footprint transform
* Added tf_prefix to urdf links
* Fixed more files
* added first summit_xl_multi.launch
* initial commit with v305 of svn
* Contributors: Dani Carbonell, Jorge Arino, JorgeArino, carlos3dx, dani-carbonell, mcantero, rguzman, trurl

* changed topic for twist output
* changed cmd_vel_out from /summit_xl_control/cmd_vel to /summit_xl/robot_control/command
* control: fixing twist_mux output topic
* control: fixing typo
* updated maitainer
* added control files for hls version
* Mux added. Old files removed.
* Removed old packages and new structure defined.
* fixing package dependencies
* summit_xl_description: adding axis sensor to summit X chasis
* adding scissor movement
* Not anymore multi.
* 1.0.0
* Fixed ptz controller error
* Allows multiple simulations of summit xl (not omni wheels yet)
* Now it uses a tf_prefix for the odom->base_footprint transform
* Added tf_prefix to urdf links
* Fixed more files
* added first summit_xl_multi.launch
* initial commit with v305 of svn
* Contributors: Dani Carbonell, Jorge Arino, JorgeArino, dani-carbonell, rguzman, trurl
