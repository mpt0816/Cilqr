#!/bin/bash

rm -rf ./core

echo "" > log.txt

sudo sysctl -p
core_dir=/home/mpt/catkin_ws/core
[ ! -d ${core_dir} ] && mkdir -p ${core_dir}
ulimit -c unlimited
sudo sysctl -w kernel.core_pattern=${core}/core.%e.%p 
sudo sysctl  -p

source devel/setup.bash
roslaunch planning pedestrian_test.launch