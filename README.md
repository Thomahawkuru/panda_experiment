
# panda_experiment

This package was written for ROS melodic running under Ubuntu 18.04. It is adapted from the panda_simulation project by Erdalpekel. The package runs a simulation of the Franka Emika Panda robot with a simulated task for teleoperation. The following Unity project is needed to run the teleoperation using a human operator:  
https://github.com/Thomahawkuru/unity_experiment

# How to install
## 1. Prerequisits

Install ros_melodic_desktop_full: 
http://wiki.ros.org/melodic/Installation/Ubuntu

then in stall needed software

```
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install ros-melodic-rosbridge-server
sudo apt-get install ros-melodic-catkin python-catkin-tools
sudo apt install python3-wstool python3-catkin-tools clang-format-10 python3-rosdep
```

## 2. Install libfranka from source
Reference: https://frankaemika.github.io/docs/installation_linux.html

Before building from source, please uninstall existing installations of libfranka and franka_ros to avoid conflicts:
```
sudo apt remove "*libfranka*"
```

To build libfranka, install the following dependencies from Ubuntu’s package manager:
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
 
Then, download the source code by cloning libfranka from GitHub:
```
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
```
In the source directory, create a build directory and run CMake:
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

## 3. Create catkin workspace and Install moveit from source
https://moveit.ros.org/install/source/
```
mkdir -p catkin_ws
cd catkin_ws
source /opt/ros/melodic/setup.bash
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
rm -rf src/panda_moveit_config/
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
```

## 4. Install experiment packages
```	
cd src
git clone https://github.com/thomahawkuru/panda_experiment.git
git clone https://github.com/thomahawkuru/panda_moveit_config.git
git clone https://github.com/thomahawkuru/franka_ros.git
cd ..
sudo apt-get install libboost-filesystem-dev
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
```

It is important that you pass the libfranka directory to catkin_make when building this ROS package
```
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.bash
```
## 5. Run the experiment:
Launch the simulation with tools for the experiment
```
roslaunch panda_experiment experiment.launch
```
Start the Whack-a-Mole task
```
roslaunch panda_experiment whack_a_mole.launch
```
You can rest the robot position during brakes and in between trials:
```
roslaunch panda_experiment reset.launch
```
