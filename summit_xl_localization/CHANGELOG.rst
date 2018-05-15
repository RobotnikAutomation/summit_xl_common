^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package summit_xl_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2018-05-15)
------------------
* localization: adding new dependencies with amcl, map_server & gmapping
* merging with kinetic-multirobot-devel
* updating mantainers
* Add parameters to change navigation and localization mode
* [summit_xl_localization] odom topic changed to robotnik_base_control/odom
* localization: adding arg to amcl.launch to use either diff or omni model type
* localization: updating map_saver.launch to save in home folder by default
* localization: updating scritpt to save map in current folder
* localization: tests scripts to print orientation values (odom,imu) as rpy
* moving amcl,map_server from nav to loc pkg. Creating arguments for most of launch files
* summit_xl_localization/navigation: changed topics and services to make work rl_utils
* [summit_xl_localization]:prefix frame added to launch files and frames
* Merge branch 'kinetic-devel' into kinetic-multirobot-devel
* Merge branch 'indigo-devel-rc' into kinetic-devel
* updated odom params for diff_drive_controller and ekf filter config
* summit_xl_localization: adds script to use current gps position to call datum service
* summit_xl_localization: adding extra info and fixes indentation
* summit_xl_localization: added wait_for_datum param
* 1.0.10

1.1.2 (2016-09-01)
------------------
* fixed merge conflicts
* summit_xl_localization: commented robot_localization launch files
* summit_xl_localization: updated robot_localization launch files
* summit_xl_localization: added navsat_transform_new to CMakeLists.txt
* 1.0.9
* updated changelog
* Contributors: Marc Bosch-Jorge, carlos3dx

1.1.1 (2016-08-24)
------------------

1.1.0 (2016-08-24)
------------------

1.0.9 (2016-08-24)
------------------
* Adding install rules
* Deleting unused files
* Added rl_utils.launch
* Contributors: Jorge Arino, summit

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
* added simple script to test magnetometer
* Merge branch 'indigo-devel' of https://github.com/RobotnikAutomation/summit_xl_common into indigo-devel
  Conflicts:
  summit_xl_localization/launch/navsat_transform_node.launch
* configuration tested in Lisbon uploaded
* summit_xl_localization: added dependency to robotnik_msgs
* Topic name change
* Minor changes
* Update launch files. Add robot_localization_utils
* Update robot_localization_odom.launch
* Frame name change
* Add local_tf node
* Static tf update
* Add launch files and the new navsat_transform node
* First commit summit_xl_localization
* Contributors: Elena Gambaro, ElenaFG, Jorge Arino, mcantero, rguzman

* added simple script to test magnetometer
* Merge branch 'indigo-devel' of https://github.com/RobotnikAutomation/summit_xl_common into indigo-devel
  Conflicts:
  summit_xl_localization/launch/navsat_transform_node.launch
* configuration tested in Lisbon uploaded
* summit_xl_localization: added dependency to robotnik_msgs
* Topic name change
* Minor changes
* Update launch files. Add robot_localization_utils
* Update robot_localization_odom.launch
* Frame name change
* Add local_tf node
* Static tf update
* Add launch files and the new navsat_transform node
* First commit summit_xl_localization
* Contributors: Elena Gambaro, ElenaFG, Jorge Arino, rguzman
