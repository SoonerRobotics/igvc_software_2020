#!/bin/bash

## udev

# copy igvc.rules to rule.d
cp etc/igvc.rules /etc/udev/rules.d/igvc.rules

# restart udev
service udev reload
sleep 2
service udev restart

## vcs
# Install vcstool
sudo pip install vcstool

# Install dependencies
vcs import < igvc.deps


# Install library dependencies
sudo apt install libspatialindex-dev

# Install python dependencies
pip install -r requirements.txt

# Install map_server dependencies
sudo apt-get install libsdl-image1.2-dev and
sudo apt-get install libsdl-dev

# costmap_2d dependencies
sudo apt-get install ros-melodic-tf2-sensor-msgs
