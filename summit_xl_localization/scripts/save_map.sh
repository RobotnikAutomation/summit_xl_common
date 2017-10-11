#!/bin/bash

MAP_FOLDER=~/catkin_ws/src/summit_xl_common/summit_xl_localization/maps

if [ $# -eq 1 ] 
	then
		echo "Saving map as $1 in $MAP_FOLDER"
		rosrun map_server map_saver -f $MAP_FOLDER/$1
else
	echo "$0: Incorrect number of params. The name of map has to be passed as argument"
fi
