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