#!/bin/bash
while true; do
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: $1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $2" -r 10
done
