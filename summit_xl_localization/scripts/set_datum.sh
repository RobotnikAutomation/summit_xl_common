#!/bin/bash
# Subscribes to /fix and calls service /datum with GPS lat/lon

latitude=$(rostopic echo -n1 /fix/latitude | head -n1)
longitude=$(rostopic echo -n1 /fix/longitude | head -n1)

$(rosservice call /datum "geo_pose:
  position:
    latitude: $latitude
    longitude: $longitude
    altitude: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0")
