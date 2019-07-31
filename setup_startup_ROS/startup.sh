#!/bin/bash
if [ "$1" == "stop" ]; then
	pids=( `ps | grep 'roslaunch\|python' | grep -o '^ [0-9]*'` )
	echo "killing ${pids[@]}"
	kill `echo ${pids[@]}`
	exit
elif [ "$1" == "start" ]; then
	echo "Starting dc_motor_driver"
	roslaunch dc_motor_driver dc_motor_driver.launch 1>/dev/null &
	sleep 3
	echo "Starting read_sonars"
	roslaunch read_sonars read_sonars.launch 1>/dev/null &
	sleep 3
	echo "Starting imu_sensor"
	rosrun read_imu read_imu.py &
	sleep 14
fi

