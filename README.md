# igvc_software_2020

Our codebase for the [IGVC 2020 Competition](http://www.igvc.org/).

## Dev Setup

Run `sudo ./dev_setup.sh` from within the `setup` directory the first time you clone this repository to install the needed dependencies, install udev rules, and more automatically. This will also provide a few useful commands to make ROS easier.

### igvc_deps

Running `igvc_deps` from anywhere will update the dependencies defined in `setup/igvc.deps`.

### igvc_make

Running `. igvc_make` from anywhere will run `catkin_make` in `igvc_ws` and then source `igvc_ws/devel/setup.bash` for you. Running `igvc_make` instead will still `catkin_make` in `igvc_ws` but not source anything.

## Robot Setup

Run `sudo ./robot_setup.sh` from within the `setup` directory. This will install udev rules, init.d rules, dependencies, and more.

## Component or System documentation

See our GitHub wiki at https://github.com/SoonerRobotics/igvc_software_2020/wiki.

## Updating dependencies

To update the dependencies in this repo, modify the `setup/igvc.deps` file, and then run `vcs import < igvc.deps` from the setup folder.

## Project Dependencies
- [vcstool](https://github.com/dirk-thomas/vcstool): we use this to manage all the external github dependencies and ROS packages we need.