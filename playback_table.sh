#!/bin/bash

move_robot(){
	stringarray=($1)
	echo "ctpq time $2 off 0 pos (${stringarray[3]} ${stringarray[2]} ${stringarray[1]})" | yarp rpc /ctpservice/torso/rpc
	echo "ctpq time $2 off 0 pos (${stringarray[@]:4:7})" | yarp rpc /ctpservice/left_arm/rpc
	echo "ctpq time $2 off 0 pos (${stringarray[@]:11:6})" | yarp rpc /ctpservice/head/rpc
}

filename=$1

{
read first_line
move_robot "${first_line}" "5"
sleep 5
n=1
while read line; do
	# reading each line
	n=$((n + 1))
	echo $n
	move_robot "${line}" "0.3"
	read -n 1 key <&1

done
} < $filename
