# ROS_Practice

Installation of ROS-Indigo on Ubuntu 14.04 LTS

## Install Ubuntu 14.04 LTS
Download from http://www.ubuntu.com/download

- Install Chrome 
    (from google page)

- Install vim
    ```
    $ sudo apt-get update
    $ sudo apt-get install vim
    ```
##Install ROS-Indigo

(see http://wiki.ros.org/indigo/Installation/Ubuntu)

1. Setup your sources.list
    ```
    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```

2. Set up your keys
    ```
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
    ```

3. Installation - Desktop-Full Install
    ```
    $ sudo apt-get update
    $ sudo apt-get install ros-indigo-desktop-full
    ```

##Setup ROS
1. Initialize ROSdep 
(rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.)
    ```
    $ sudo rosdep init
    $ rosdep update
    ```

2. Environment setup
    ```
    $ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```

3. Getting rosinstall
    ```
    $ sudo apt-get install python-rosinstall
    ```

##Create and Setup a ROS Workspace
(see http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

1. Create the workspace
    ```
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
    ```

2. Build the workspace
    ```
    $ cd ~/catkin_ws/
    $ catkin_make
    ```

3. Source the setup.bash in devel/ (echo into the .bashrc so that we won't need to source it every time we startup the system)
(see "Mastering ROS for Robotic Programing", pp.27)
    ```
    $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```

##Installation of Some ROS Packages

1. Gazebo-ROS Interface
    ```
    $ sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-msgs ros-indigo-gazebo-plugins ros-indigo-gazebo-ros-control
    ```
    
    Models for Gazebo 2.2
    (see http://answers.ros.org/question/199401/problem-with-indigo-and-gazebo-22/)
    
    ```
    $ wget -r -R "index\.html*" http://models.gazebosim.org/
    ```
    
    Copy them (the download place is inside `~/.gazebo/`) into `~/.gazebo/models` folder (after you mkdir models inside `~/.gazebo`) and everything will be ok.

2. Moveit!
    ```
    $ sudo apt-get install ros-indigo-moveit-full
    ```

3. Navigation Stack
    ```
    $ sudo apt-get install ros-indigo-navigation
    ```
4. Gmapping
    ```
    $ sudo apt-get install ros-indigo-gmapping
    ```

5. Joint_state_controller
    ```
    $ sudo apt-get install ros-indigo-joint-state-controller
    ```

6. Joy (joystick)
    ```
    $ sudo apt-get install ros-indigo-joy
    ```
7. USB camera
    ```
    $ sudo apt-get install ros-indigo-usb-camera
    ```

##Git Setup
1. Initiate the git in Ubuntu
    ```
    $ git config --global user.name "Benson516"
    $ git config --global user.email "benson516@hotmail.com"
    ```
    
2. Clone this repository under **/catkin_ws**
    ```
    $ cd /catkin_ws
    $ git clone https://github.com/Benson516/ROS_Practice.git
    ```
    
3. Rename the directory to **src**
    ```
    $ mv ROS_Practice src
    ```

##Get Files and Examples from Github

1. My Programs for practicing ROS at github
    ```
    $ cd catkin_ws
    $ git clone https://github.com/Benson516/ROS_Practice.git
    ```

2. Examples in "Mastering ROS for Robotic Programing"
    ```
    $ cd ~
    $ git clone https://github.com/Benson516/mastering_ros.git
    ```
