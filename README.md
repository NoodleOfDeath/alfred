# ALFRED (Autonomous Labor Focused ROS Extensible Drone)

Just a fun project that allows you to control an iRobot Create 2 from a React.js web application, a native mobile application, a wireless Xbox controller, or even a smart watch. 

The primary purpose of this project is to demonstrate a simple and foundational full-stack design of a ROS extensible ecosystem where robots on the same network can talk to one other as well as be controlled by any RESTful UI application.

## Getting Started

### Materials

* An [iRobot Create 2](https://store.irobot.com/default/create-programmable-programmable-robot-irobot-create-2/RC65099.html)
* A [Raspberry Pi 4](https://www.amazon.com/gp/product/B07TC2BK1X/ref=ppx_yo_dt_b_asin_title_o09_s00?ie=UTF8&psc=1), hereby referred to as __Alfred__
* A separate machine (hereby referred to as __The Batcomputer__) that can support the Robot Operating System (ROS); ROS2 is not yet widely used enough for all of this project's dependencies to be built.
  * My host OS is Mac Monterey, which only suports ROS2, so I used a [VirtualBox](https://www.virtualbox.org/wiki/Downloads) virtual machine, which can work as long you use a "bridged network adapter", a physical ethernet cable connection to your host machine, and explicitly map hostnames to local IP addresses on each device correctly.
  * All devices need to be on the same local network for the ROS messaging system to work properly.
* Xcode (if you own a Mac) is optional should you wish to run the iOS/SmartWatch POC.

### Setup

#### Install ROS on each Machine

* [Install Ubuntu Server 20.04 on your Raspberry Pi 4](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview)
  * __Make sure you install Ubuntu Server, and not Raspbian or Ubuntu Desktop__
* [Install ROS on the Raspberry Pi 4 over SSH](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview)
  * The Noetic distribution of ROS was used for this project
  * __Be sure to use the same version/distribution of ROS on ALL machines that will be part of the ROS network__
* [Install Ubuntu 20.04 on The Batcomputer](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
  * If you decide to use a virtual machine, like I did, make sure to connect an actual ethernet cable to your host machine, and change the network adapter for VM to be a "bridged network"
* [Install ROS on The Batcomputer](http://wiki.ros.org/Installation/Ubuntu)

#### Configure Environments

* Find the local IP address of Alfred and The Batcomputer with `ifconfig`. It should look something like `192.168.x.1` or `192.168.x.2`.
* Edit the hostname mappings on Alfred with `sudo nano /etc/hosts` and make sure you have the following lines:
```bash
127.0.0.1    localhost
127.0.1.1    alfred
192.168.x.2  batcomputer
```
* On Alfred add the following lines to the end of `~/.bashrc` then after you save run `source ~/.bashrc`:
```bash
source /opt/ros/noetic/setup.bash
export ROS_HOSTNAME=alfred
export ROS_MASTER_URI=http://batcomputer:11311
```
* Edit the hostname mappings on The Batcomputer with `sudo nano /etc/hosts` and make sure you have the following lines:
```bash
127.0.0.1    localhost
127.0.1.1    batcomputer
192.168.x.1  alfred
```
* On The Batcomputer add the following lines to the end of `~/.bashrc` then after you save run `source ~/.bashrc`:
```bash
source /opt/ros/noetic/setup.bash
export ROS_HOSTNAME=batcomputer
export ROS_MASTER_URI=http://batcomputer:11311
```

To verify if your ROS environments are configured correctly:

* Launch ROS Core on The Batcomputer by just running the command `roscore`
* On Alfred run the command `rostopic echo /batstuff`
* Finally, in a second terminal window on The Batcomputer run the command `rostopic pub -r 10 /batstuff std_msgs/String "I'll be home late, Alfred"`
  
If you have configured your ROS environments correctly, the terminal on Alfred should start receiving messages from The Batcomputer at a rate of 10Hz.

### Building the Code

#### Build ROS Packages on Alfred

* In order to build the ROS packages used by Alfred, you will need to move the `src` directory into your ROS workspace, which is typically `~/ros_catkin_ws`. Create the workspace directory if you need to: `mkdir -p ~/ros_catkin_ws/src`.
```bash
ubuntu@alfred$ mkdir ~/opt
ubuntu@alfred$ cd ~/opt
ubuntu@alfred$ git clone https://github.com/NoodleOfDeath/alfred
ubuntu@alfred$ cp -rf alfred/src/alfred ~/ros_catkin_ws/src
```
* Next, you'll need to clone the ROS package projects for the iRobot Create 2
```bash
ubuntu@alfred$ cd ~/ros_catkin_ws/src
ubuntu@alfred$ git clone https://github.com/AutonomyLab/libcreate.git
ubuntu@alfred$ git clone https://github.com/AutonomyLab/create_robot.git
ubuntu@alfred$ ls
alfred create_robot libcreate
```
* Next use catkin tools to build all of the ROS packages on Alfred:
```bash
ubuntu@alfred$ cd ~/ros_catkin_ws
ubuntu@alfred$ catkin build
ubuntu@alfred$ source devel/setup.bash
```

#### Build Custom ROS .msg Package on The Batcomputer

### Run the Application Stack

#### Start Up the FastAPI Instance on The Batcomputer

#### Run the React.js Web Application on The Batcomputer

#### Control the Robot from the React.js Web Application

#### Control the Robot using the iOS Application
