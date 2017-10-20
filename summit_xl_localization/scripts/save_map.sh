#!/bin/bash

MAP_FOLDER='./'

if [ $# -eq 1 ] 
	then
		echo "Saving map as $1 in $MAP_FOLDER"
		rosrun map_server map_saver -f $1
else
	echo "$0: Incorrect number of params. The name of map has to be passed as argument"
fi
