# DVRK LATEST STABLE VERSION

- dVRK 2.2.1

- cisst 1.2.1

## Deps

- Ubuntu 20.04

- ROS Noetic

``` bash
sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev libhidapi-dev python3-pyudev python3-vcstool python3-catkin-tools python3-osrf-pycommon python-is-python3 python3-catkin-tools

```

## How To

- Create your own *catkin_ws*

``` bash
mkdir -p dvrk_ws/src
``` 

- Clone the repository inside src folder

``` bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

Then, you can source your *catkin_ws* 

``` bash
source dvrk_src/devel/setup.bash
``` 

