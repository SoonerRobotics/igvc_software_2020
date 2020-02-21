#!/bin/bash

## udev

# copy igvc.rules to rule.d
cp etc/igvc.rules /etc/udev/rules.d/igvc.rules
cp etc/51-kinect.rules /etc/udev/rules.d/51-kinect.rules

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

## Kinect deps

# build lib
cd ../igvc_ws/src/deps/libfreenect
mkdir build
cd build
cmake -L ..
make
sudo make install
sudo ldconfig /usr/local/lib64/

# install deps
cd ../wrappers/python
sudo apt-get install cython -y
sudo apt-get install python-dev -y
sudo apt-get install python-numpy -y
sudo python setup.py install

# add user permissions
sudo adduser $USER video
sudo adduser $USER plugdev