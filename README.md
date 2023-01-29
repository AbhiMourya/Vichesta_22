# Vichesta (Takshak'22)

## Supported OS
> **Ubuntu** _(18.04 or 20.04)_
>> Ubuntu 20.04 is recommended

## Installation
You need to install ROS for running the packages in your device.

> **_ROS Noetic is recommended_** for running the packages, but you may try ROS melodic too.
> ROS Noetic requires Ubuntu 20.04

### Steps:

#### ROS
Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Set up your keys

```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

First, make sure your Debian package index is up-to-date:

```
sudo apt update
```

Install ROS-NOETIC Full: Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages

```
sudo apt install ros-noetic-desktop-full
```
Environment setup
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Dependencies for building packages
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
#### Installation of Moveit for ROS Noetic
Install Moveit completely:
```
sudo apt install ros-noetic-moveit*
```

#### ROS Package on which our workspace depend:
```
sudo apt install ros-noetic-joint* ros-noetic-controller* ros-noetic-rqt-controller-manager ros-noetic-rqt-joint-trajectory-controller
```

#### Event Package
Open up the terminal and follow the following commands:
```
cd
mkdir vichesta_ws
cd vichesta_ws
mkdir src
cd src
git clone https://github.com/Tanishq30052002/Vichesta-22-Takshak.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

Source the workspace in .bashrc
> ```
> cd
> echo "source ~/vichesta_ws/devel/setup.bash" >> ~/.bashrc
>  ```

Add models to be used in gazebo simulation in .gazebo/models
```
roscd vichesta_22/models
cp -R ./* ~/.gazebo/models/
```

#### Run the launch file to open up the enviroment.
```
roslaunch vichesta_22 vichesta_task.launch
```

##### For Starting and Stopping Conveyor use the following commands:
> Start Red
> ```
> rosservice call /red_conveyor/conveyor/control "power: 100.0"
> ```
> Start Green
> ```
> rosservice call /green_conveyor/conveyor/control "power: 100.0"
> ```
> Stop Red
> ```
> rosservice call /red_conveyor/conveyor/control "power: 0.0"
> ```
> Stop Green
> ```
> rosservice call /green_conveyor/conveyor/control "power: 0.0"
> ```

#### Enviroment should look like:

Images

![RVIZ](https://user-images.githubusercontent.com/78026976/210094444-f32d052b-1b28-4058-9124-79632eb4d9f0.png)

![Gazebo_1](https://user-images.githubusercontent.com/78026976/210094408-a18bd3e2-db3e-4b1a-b3f0-7fdcf4184278.png)
![Gazebo_2](https://user-images.githubusercontent.com/78026976/210094427-4538d2d7-7feb-4e29-8da8-145bac5301b2.png)

Video

https://user-images.githubusercontent.com/78026976/210094438-2821b4e1-0091-4029-8713-e065c2fff305.mp4

## For Problem Statement and Rulebook:
Problem Statement

Rulebook
