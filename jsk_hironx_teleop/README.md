The master TouchHID PC is configured with ROS noetic, and the slave HIRO PC is configured with melodic.

## (i) Establishment of environment for TouchHID operation PC(master)
### (a) Touch driver installation
- Install OpenHaptics Installer and Haptic device driver based on the official TouchHID website(https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US).

- Backup: https://drive.google.com/drive/folders/1SiLa0ZBKllt6mm_c99FPo-aZoehjO0P2

- As of April 9, 2024, the version of Qt installed by apt-get install qt5-default is 5.12.8 on Ubuntu 20.04, but this causes an error. If you install 5.12.0 with the following command and refer to the lib of Qt5.12.0 in `LD_LIBRARY_PATH`, the error will be resolved.

```
wget https://download.qt.io/archive/qt/5.12/5.12.0/qt-opensource-linux-x64-5.12.0.run
chmod +x qt-opensource-linux-x64-5.12.0.run
./qt-opensource-linux-x64-5.12.0.run
```
Example: `LD_LIBRARY_PATH=(path/to/QT5.12.0)/Qt5.12.0/5.12.0/gcc_64/lib . /bin/Touch_Diagnostic` (depending on where the library is located)

See https://github.com/jsk-ros-pkg/jsk_robot/issues/1907 for more information.


### (b) Create and build a ROS workspace
Type the following command in the ROS noetic environment.

```
source /opt/ros/noetic/setup.bash
mkdir -p ~/touch_ws/src
cd ~/touch_ws/src
wstool init
```

Edit `.rosinstall` as follows:
```
git:
    local-name: franka_ros
    uri: https://github.com/pazeshun/franka_ros.git
    version: install-FrankaCombinableHW
git:
    local-name: jsk-ros-pkg/jsk_recognition
    uri: https://github.com/jsk-ros-pkg/jsk_recognition.git
    version: f455d96f431d094177be5f7569c79a3e25e8e360
git:
    local-name: jsk-ros-pkg/jsk_hironx_teleop
    uri: https://github.com/YUKINA-3252/jsk_hironx_teleop.git
    version: jsk_hironx_teleop
git:
    local-name: phantom_drivers
    uri: https://github.com/pazeshun/Geomagic_Touch_ROS_Drivers.git
    version: dual-phantom-readme
```
Then build:

```
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ..
catkin build
```

## (ii) HIRO operation PC (slave) environment construction
Create a workspace and execute the wstool command as in (i). On the slave side, edit `.rosinstall` as follows and build.
```
git:
    local-name: franka_ros
    uri: https://github.com/pazeshun/franka_ros.git
    version: install-FrankaCombinableHW
- git:
    local-name: jsk-ros-pkg/jsk_control
    uri: git@github.com:jsk-ros-pkg/jsk_control.git
- git:
    local-name: jsk-ros-pkg/jsk_recognition
    uri: https://github.com/jsk-ros-pkg/jsk_recognition.git
    version: f455d96f431d094177be5f7569c79a3e25e8e360
- git:
    local-name: jsk-ros-pkg/jsk_robot
    uri: https://github.com/jsk-ros-pkg/jsk_robot.git
    version: master
- git:
    local-name: libfranka
    uri: https://github.com/frankaemika/libfranka
- git:
    local-name: multisense
    uri: https://github.com/Naoki-Hiraoka/multisense_ros
    version: 3.4.9_fixed
- git:
    local-name: phantom_drivers
    uri: https://github.com/pazeshun/Geomagic_Touch_ROS_Drivers.git
    version: dual-phantom-readme
- git:
    local-name: jsk_hironx_teleop
    uri: https://github.com/YUKINA-3252/jsk_hironx_teleop.git
- git:
    local-name: rtmros_hrp2
    uri: https://github.com/start-jsk/rtmros_hrp2
    version: master
- git:
    local-name: rtmros_tutorials
    uri: https://github.com/start-jsk/rtmros_tutorials
    version: master
- git:
    local-name: jsk-ros-pkg/jsk_common
    uri: https://github.com/jsk-ros-pkg/jsk_common.git
    version: master
```

## (iii) How to operate
### Master side
Publish the state of the Touch device as a ROS topic in `start_hironx_teleop_master_side.launch`.
```
LD_LIBRARY_PATH=(path/to/QT5.12.0)Qt5.12.0/5.12.0/gcc_64/lib:$LD_LIBRARY_PATH roslaunch jsk_hironx_teleop start_hironx_teleop_master_side.launch
```

### Slave side
The operation is performed from roseus.

```
cd ~/touch_ws/src/jsk_hironx_teleop/jsk_hironx_teleop/euslisp
roseus
load "phantom_to_hiro_without_tf_convert.l"
```