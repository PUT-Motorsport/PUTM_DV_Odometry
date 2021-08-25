#!/bin/bash

modprobe can_dev
modprobe can_raw
modprobe vcan
ip link add dev vcan0 type vcan
ip link set up vcan0
docker run -it -v $VOLUME:/shared --env="DISPLAY" --net=host --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --privileged ros:latest
