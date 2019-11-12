# igvc_software_2020

Our codebase for the [IGVC 2020 Competition](http://www.igvc.org/).

## First Time Setup

Please run `sudo ./dev_setup.sh` from within the `setup` directory the first time you clone this repository to install the needed dependencies, install udev rules, and more automatically. This will also provide a few useful commands to make ROS easier.

### igvc_deps

Running `igvc_deps` from anywhere will update the dependencies defined in `setup/igvc.deps`.

### igvc_make

Running `. igvc_make` from anywhere will run `catkin_make` in `igvc_ws` and then source `igvc_ws/devel/setup.bash` for you. Running `igvc_make` instead will still `catkin_make` in `igvc_ws` but not source anything.

## Component or System documentation

See our GitHub wiki at https://github.com/SoonerRobotics/igvc_software_2020/wiki.

## Updating dependencies

To update the dependencies in this repo, modify the `setup/igvc.deps` file, and then run `vcstool import < igvc.deps` from the setup folder.