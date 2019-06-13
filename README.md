# Udacity Self Driving Car Nanodegree Term3 System Integration Project Introduction

### 1. Algorithm
Whole algorithm is based on ROS system. There are three main part.
1. waypoint updater node 
2. traffic light state process node
3. drive by wire node. 
![whole process](https://github.com/Fred159/CarND-Term3-System-Integration-Project/blob/master/imgs/final-project-ros-graph-v2.png)

The code structure is shown as below.
![Code Structure of Project](https://github.com/Fred159/CarND-Term3-System-Integration-Project/blob/master/resultsIMG/Code_structure.png)

### 2. Issues with udacity original repository 
- I think In system integration project. The message type 'float32 steering_wheel_angle_cmd' should be change into 'float32 steering_wheel_cmd' . Because I think as link shows there are no message named with 'float32 steering_wheel_angle_cmd' (refer to [this link](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/default/dbw_mkz_msgs/msg/SteeringReport.msg))

- Errors like below : 
Traceback (most recent call last):
  File "/opt/ros/kinetic/bin/catkin_make", line 13, in <module>
    from catkin.terminal_color import disable_ANSI_colors, fmt
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/catkin/terminal_color.py", line 2, in <module>
    from catkin_pkg.terminal_color import *  # noqa
ImportError: No module named terminal_color
 can fixed by this command in terminal : pip install --upgrade catkin_pkg_modules
  
  - https://github.com/Praveenraj49/CarND-Capstone/blob/master/README.md for dbw_mkz_msgs_DIR error
  
  
### 3. Results
The simulation results shows that algorithm works appropriately. Sometimes I think the PID controller should be tuned more.

- Success build
![Success build](https://github.com/Fred159/CarND-Term3-System-Integration-Project/blob/master/resultsIMG/Success_run.png)

- Initialization
![initialization](https://github.com/Fred159/CarND-Term3-System-Integration-Project/blob/master/resultsIMG/Initialize_state.png)

- Stop at red line
![Stop at red line](https://github.com/Fred159/CarND-Term3-System-Integration-Project/blob/master/resultsIMG/Stop_at_stopline_redlight.png)

- Go when light change into green
![Go when green light](https://github.com/Fred159/CarND-Term3-System-Integration-Project/blob/master/resultsIMG/Move_when_lightgreen.png)

#### Youtube video link
[![Youtube Video of simulation](https://github.com/Fred159/CarND-Term3-System-Integration-Project/blob/master/resultsIMG/Initialize_state.png)](https://youtu.be/co9v8uI5phI "Simulation results")

### 4. Other things to do
- Recognize the traffic light like in real world
- Combine the code in to carla which is real car in udacity
- Figure out the compuation load.
- Optimization should be done in path planning.

# Original Udacity introduction
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
