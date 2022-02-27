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
export ROS_HOSTNAME=192.168.x.2
export ROS_MASTER_URI=http://batcomputer:11311
```

NOTE that `ROS_HOSTNAME` is set to the actual local IP address of Alfred, because otherwise, sensor topics won't be connected properly.

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
* Finally, in a second terminal window on The Batcomputer run the following command:

```bash
ubuntu@batcomputer$ rostopic pub -r 10 /batstuff std_msgs/String "I'll be home late, Alfred"
```
  
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

* Use `rosdep` to download dependencies and then catkin tools to build all of the ROS packages on Alfred:

```bash
ubuntu@alfred$ cd ~/ros_catkin_ws
ubuntu@alfred$ rosdep install --from-paths src
ubuntu@alfred$ rosdep update
ubuntu@alfred$ catkin build
...                                                                                                                        
[build] Summary: All 7 packages succeeded!                                                                                                                                                 
[build]   Ignored:   None.                                                                                                                                                                 
[build]   Warnings:  None.                                                                                                                                                                 
[build]   Abandoned: None.                                                                                                                                                                 
[build]   Failed:    None.                                                                                                                                                                 
[build] Runtime: 13.3 seconds total.
```

* Once the packages build successfully, plugin the USB cable for the iRobot Create 2 into the Raspbeery Pi, then run the following command on Alfred and you should hear the robot boot up:

```bash
ubuntu@alfred$ cd ~/ros_catkin_ws
ubuntu@alfred$ source devel/setup.bash
ubuntu@alfred$ roslaunch alfred alfred.launch
... logging to /home/ubuntu/.ros/log/7000dcd8-94b6-11ec-a63c-ebe27d46d47f/roslaunch-alfred-15936.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://alfred:45035/
...
[ WARN] [1645649489.650718343]: link 'base_footprint' material 'Green' undefined.
[ WARN] [1645649489.666563810]: link 'base_footprint' material 'Green' undefined.
[ INFO] [1645649489.690201817]: CreateDriver Online
[ INFO] [1645649489.695118354]: [CREATE] "CREATE_2" selected
[ INFO] [1645649490.885812415]: [CREATE] Connection established.
[ INFO] [1645649490.886188947]: [CREATE] Battery level 33.51 %
[ INFO] [1645649491.466942727]: [CREATE] Ready.
```

#### Build Custom ROS .msg Package on The Batcomputer

A custom ROS msg format is used called `Float32List`. This package needs to be build on both Alfred (as you did already) and The Batcomputer. Copy the package source into the ROS workspace on The Batcomputer and simply run `catkin build`:

```bash
ubuntu@batcomputer$ mkdir -p ~/ros_catkin_ws/src
ubuntu@batcomputer$ mkdir ~/opt
ubuntu@batcomputer$ cd ~/opt
ubuntu@batcomputer$ git clone https://github.com/NoodleOfDeath/alfred
ubuntu@batcomputer$ cp -rf alfred/api/batcomputer/ros_pkgs/batcomputer ~/ros_catkin_ws/src
ubuntu@batcomputer$ cd ~/ros_catkin_ws
ubuntu@batcomputer$ catkin build
```

### Run the Application Stack

The Batcomputer will be running ROS core, a FastAPI server, and a React.js web application.

* Make sure ROS core is running on The Batcomputer by running `roscore` in the terminal.
* In a second tab, run the subscriber node on The Batcomputer by running

```bash
ubuntu@batcomputer$ source cd ~/ros_catkin_ws/devel/setup.bash
ubuntu@batcomputer$ cd ~/opt/alfred/api/batcomputer
ubuntu@batcomputer$ ./sub.py
```

* Finally, in a third terminal tab, start the FastAPI instance on The Batcomputer either as a docker container or directly with `uvicorn`

```bash
ubuntu@batcomputer$ source cd ~/ros_catkin_ws/devel/setup.bash
ubuntu@batcomputer$ cd ~/opt/alfred/api/batcomputer
ubuntu@batcomputer$ uvicorn api:app --reload --host "0.0.0.0"
INFO:   Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:   Started reloader process [11995]
INFO:   topicmanager initialized
```

* Update node on The Batcomputer to 14+ or the React.js application will not work.

```bash
ubuntu@batcomputer$ sudo apt update
ubuntu@batcomputer$ curl -sL https://deb.nodesource.com/setup_14.x | sudo bash -
ubuntu@batcomputer$ cat /etc/apt/sources.list.d/nodesource.list
deb https://deb.nodesource.com/node_14.x focal main
deb-src https://deb.nodesource.com/node_14.x focal main
ubuntu@batcomputer$ sudo apt -y install nodejs
```

* Run the React.js web application on The Batcomputer

```bash
ubuntu@batcomputer$ cd ~/opt/alfred/ui/react
ubuntu@batcomputer$ npm i --save
ubuntu@batcomputer$ npm start
Compiled successfully!

You can now view batcomputer in the browser.

  Local:            http://localhost:3000
  On Your Network:  http://batcomputer:3000

Note that the development build is not optimized.
To create a production build, use npm run build.
```

* To control the robot from the React.js web application, open a browser on The Batcomputer and navigate to [http://batcomputer:3000](http://batcomputer:3000).

You should be able to drive and rotate the robot with this web application.

To control the robot with an iOS application, open the Xcode project on a Mac and simply run it in the simulator or on a device on the same local network.

