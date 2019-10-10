#!/bin/bash

# copy igvc.rules to rule.d
cp igvc.rules /etc/udev/rules.d/igvc.rules

# restart udev
service udev reload
sleep 2
service udev restart
