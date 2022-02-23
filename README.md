# Alfred (Autonomous Labor Focused ROS Extensible Drone)

Just a fun project that allows you to control an iRobot Create 2 from React.js, native mobile application, a wireless Xbox controller, and even a smart watch.

## Getting Started

### Materials

* An [iRobot Create 2](https://store.irobot.com/default/create-programmable-programmable-robot-irobot-create-2/RC65099.html)
* A [Raspberry Pi 4](https://www.amazon.com/gp/product/B07TC2BK1X/ref=ppx_yo_dt_b_asin_title_o09_s00?ie=UTF8&psc=1), hereby referred to as __Alfred__
* A separate machine (hereby referred to as __The Batcomputer__) that can support the Robot Operating System (ROS); ROS2 is not yet widely used enough for all of this project's dependencies to be built.
  * My host OS is Mac Monterey, which only suports ROS2, so I used a [VirtualBox](https://www.virtualbox.org/wiki/Downloads) virtual machine, which can work as long you use a "bridged network adapter", a physical ethernet cable connection to your host machine, and explicitly map hostnames to local IP addresses on each device correctly.
  * All devices need to be on the same local network for the ROS messaging system to work properly.
* Xcode (if you own a Mac) is optional should you wish to run the iOS/SmartWatch POC.

### Setup

#### Configure and Install ROS on each Machine

* [Install Ubuntu Server 20.04 on your Raspberry Pi 4](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview)
  * __Make sure you install Ubuntu Server, and not Raspbian or Ubuntu Desktop__
* [Install ROS on the Raspberry Pi 4 over SSH](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview)
  * The Noetic distribution of ROS was used for this project
  * __Be sure to use the same version/distribution of ROS on ALL machines that will be part of the ROS network__
* [Install Ubuntu 20.04 on The Batcomputer](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
  * If you decide to use a virtual machine, like I did, make sure to connect an actual ethernet cable to your host machine, and change the network adapter for VM to be a "bridged network"
* [Install ROS on The Batcomputer]
  
### Building the Code

#### Build ROS Packages on Alfred

To build the ROS packages used by Alfred, you will need to move the `src` directory into your ROS workspace, which is typically `~/ros_catkin_ws`. Create the workspace directory if you need to: `mkdir -p ~/ros_catkin_ws/src`.

```bash
$ source /opt/ros/noetic/setup.bash
$ mkdir ~/opt
$ cd ~/opt
$ git clone https://github.com/NoodleOfDeath/alfred
$ cp -rf alfred/src/alfred ~/ros_catkin_ws/src
```

Next, you'll need to download the ROS package projects for the iRobot Create 2

```bash
$ cd ~/ros_catkin_ws/src
$ git clone https://github.com
$
```