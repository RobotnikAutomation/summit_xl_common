^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package summit_xl_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2018-05-15)
------------------
* navigation: adding dependencies with move_base & costmap_prohibition_layer
* merging with kinetic-multirobot-devel
* updating mantainers
* Add Steel teb parameters file
* Update teb config and remove unused files
* Add costmap_prohibition_layer dependency
* Add parameters to change navigation and localization mode
* Added teb_local_planner and costmap_prohibition_layer as Required Components
* [summit_xl_navigation] odom topic changed to robotnik_base_control/odom
* Add missing dependencies
* removed maps subfolder of CMakeLists
* navigation: added docking launch for steel
* moving amcl,map_server from nav to loc pkg. Creating arguments for most of launch files
* navigation: adding default arguments to slam_gmapping launch file
* move_base_nomap modified to parametrize the params of yaml
* deleted move_base.launch
* [summit_xl_navigation]:params of move_base yaml parametrized to work with multirobot
* summit_xl_localization/navigation: changed topics and services to make work rl_utils
* [summit_xl_navigation]:added prefix to launch file
* [summit_xl_navigation]: frame updated with prefix
* [summit_xl_navigation]:prefix frame added to launch files and frames
* xacro updated to multirobot
* Merge branch 'kinetic-devel' into kinetic-multirobot-devel
* Merge branch 'indigo-devel-rc' into kinetic-devel
* Merge branch 'indigo-devel' into indigo-devel-rc
* Merge branch 'indigo-devel-rc' into kinetic-devel
* summit_xl_navigation map model added
* yaml,launch and xacro modified to multirobot
* summit_xl_navigation: corrected docking launch files
* summit_xl_navigation: added docking launch files
* updated navigation files
* corrected map and laser link according to new name in robotnik_sensors
* summit_xl_navigation: fixes map publish_frequency
* summit_xl_navigation: adds configuration file for teb_local_planner in holonomic configuration
* summit_xl_navigation: updating package.xml
* summit_xl_navigation: using teb_local_planner as default planner
* summit_xl_navigation: fix inflation_radius
* Adds rbk_warehouse map
* 1.0.10

1.1.2 (2016-09-01)
------------------
* fixed merge conflicts
* 1.0.9
* updated changelog
* Contributors: Marc Bosch-Jorge, carlos3dx

1.1.1 (2016-08-24)
------------------

1.1.0 (2016-08-24)
------------------

1.0.9 (2016-08-24)
------------------
* Deleting unused files
* Contributors: Jorge Arino

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
* added move_base_nomap.launch
* created maps folder
* updated to current topics and links
* Frame name change
* Update rviz_satellite.launch
* Update rviz_satellite.rviz
* Add rviz_satellite launch file
* updated parameters
* changed config files to match rb1
* modified launch files
* added some launch files and config files
* Pad bug solved. Now it commands 0.0 when DeadM button is released. Other minor changes
* Now summit_xl_pad only publishes when deadman's buttos is pressed.
* Contributors: Dani Carbonell, Elena Gambaro, ElenaFG, dani-carbonell, mcantero, rguzman

* added move_base_nomap.launch
* created maps folder
* updated to current topics and links
* Frame name change
* Update rviz_satellite.launch
* Update rviz_satellite.rviz
* Add rviz_satellite launch file
* updated parameters
* changed config files to match rb1
* modified launch files
* added some launch files and config files
* Pad bug solved. Now it commands 0.0 when DeadM button is released. Other minor changes
* Now summit_xl_pad only publishes when deadman's buttos is pressed.
* Contributors: Dani Carbonell, Elena Gambaro, ElenaFG, dani-carbonell, rguzman
