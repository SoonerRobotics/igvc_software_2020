# igvc_software_2020

Our codebase for the [IGVC 2020 Competition](http://www.igvc.org/).

## First Time Setup

Please run `sudo ./run.sh` from within the `setup` directory the first time you clone this repository to install the needed dependencies, install udev rules, and more automatically.

## Component or System documentation

See our GitHub wiki at https://github.com/SoonerRobotics/igvc_software_2020/wiki.


## Updating dependencies

To update the dependencies in this repo, modify the `setup/igvc.deps` file, and then run `vcstool import < igvc.deps` from the setup folder.