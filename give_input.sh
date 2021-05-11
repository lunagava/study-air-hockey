#!/bin/bash



{
sleep 5

while read line; do
	# reading each line
	
	echo "ctpq time 5 off 0 pos -1" | yarp write ... /study-air-hockey/target 
	sleep 5
	echo "ctpq time 5 off 0 pos 0" | yarp write ... /study-air-hockey/target 
	sleep 5
	echo "ctpq time 5 off 0 pos 1" | yarp write ... /study-air-hockey/target 
	sleep 5
	echo "ctpq time 5 off 0 pos -1" | yarp write ... /study-air-hockey/target 
	sleep 5
done
} 
