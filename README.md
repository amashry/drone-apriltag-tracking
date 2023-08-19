# Drone AprilTag Tracking

## Introduction

This project focuses on integrating and simulating AprilTag tracking using ROS and Gazebo. This setup allows to simulate a drone with camera sensors detecting AprilTags in a simulated Gazebo environment. This README provides all the steps needed to set up, simulate, and visualize the results. The simulated results has been tested on an actual drone (MODALAI M500) as shown below. The simulations has been generated in Ubuntu 20.04 with ROS Noetic, Gazebo Classic and PX4 13.3.  

## Results

### 1. Gazebo Simulation
The following is a short GIF showing the drone following a moving apriltag in Gazebo simulation (played at 3x speed). 

![Apriltag_Tracking_Simulation](https://github.com/amashry/drone-apriltag-tracking/assets/98168605/edb0ec1d-e823-4755-9bbb-bb38ebbbe4b3)

[Watch Full Simulation Video at Regular Speed](https://drive.google.com/file/d/1BENt4jlVE9ZlOnhGOyNCU46K_hcbx11J/view?usp=sharing)

### 2. Real-World Testing on ModalAI M500 Drone (2x speed)

https://github.com/amashry/drone-apriltag-tracking/assets/98168605/4e1bd80d-7f1c-4468-b8f3-5dceb8ecf97a

## Prerequisites

- ROS
- Gazebo Classic
- [`PX4 Autopilot`](https://github.com/PX4/PX4-Autopilot)
- [`apriltag ros`](https://github.com/AprilRobotics/apriltag_ros) 
- [`image pipeline`](https://github.com/ros-perception/image_pipeline) 
- [`gazebo_apriltag`](https://github.com/koide3/gazebo_apriltag) 

## Directory Structure

```
drone-apriltag-tracking/
│
├── config/
│   ├── tags.yaml
│   ├── settings.yaml
│   └── gazebo_camera_cal.yaml
│
├── launch/
│   ├── apriltag_tracking.launch
│   ├── gazebo_detection.launch
│   ├── image_proc.launch
│   └── sitl_gazebo_custom.launch
|
|── models/
|   |── fpv_cam
|   |── iris_fpv_cam
|
├── scripts/
│   └── pub_camera_info.py
│
└── src/
|   └── apriltag_tracking.cpp
|   └── sample.cpp
|
├── worlds/
│   └── empty_apriltag.world
│
```

## Configuration

1. **tags.yaml**: Contains definitions of tags to detect. Adjust the size, name, or add other tags as required.
2. **settings.yaml**: Parameters for AprilTag 3 code. No changes are required for this project.
3. **gazebo_camera_cal.yaml**: Contains camera intrinsics for image rectification and publishing camera info as a ROS topic.

## Steps to Set Up and Run the Project

### 1. Launch Gazebo with Desired Drone Model and Environment

1. Navigate to the cloned PX4 Autopilot directory and set up the environment:

    ```bash
    cd <PX4-Autopilot_clone>
    DONT_RUN=1 make px4_sitl_default gazebo-classic
    source ~/catkin_ws/devel/setup.bash
    source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
    ```

2. Specify vehicle and world in the custom `sitl_gazebo.launch` file:

    ```bash
    <!-- vehicle model and world -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty_custom.world"/>
    <arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/iris_fpv_cam/iris_fpv_cam.sdf"/>
    <env name="PX4_SIM_MODEL" value="gazebo-classic_$(arg vehicle)" />
    ```

3. Clone the Gazebo AprilTags into your Gazebo models directory. Then, modify your `.world` file to include the AprilTag model:

    ```bash
    </physics>
    <include>
        <uri>model://Apriltag36_11_00000</uri>
        <pose>0 0 0 0 0 0</pose>
    </include>
    </world>
    </sdf>
    ```
4. INSTEAD, just use the adjusted launch file with the models and world files included. Launch the simulation:

    ```bash
    roslaunch sitl_gazebo_custom.launch
    ```
This launches PX4 SITL, Gazebo environment, and spawns vehicle. The only change is that the world used in this launch file is the world file that has the apriltag, which is added in the worlds directory for reference.

5. Optionally, for custom camera calibration, follow the below steps using the checkerboard generator and ROS calibration:
  - Download the checkerboard generator at https://gist.github.com/Kukanani/4b09debf29eafdd4d96c4717520e6f18
  - Generate a checkerboard and save the model folder in: `/PX4Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models`
  - Add this model also to .world file, and offset from the apriltag.
  - Then use the the checkerboard to calibrate the camera following this http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

### 2. Control the Drone in Simulation using QGC

- Set the UDP port for QGC communication with the PX4 simulator to **14550**.
- When working with an actual drone, adjust the port to **14551** in the launch file and QGC settings.

### 3. Publish Camera Info

In a new terminal, navigate to the scripts directory and execute:

```bash
python3 pub_camera_info.py ../config/gazebo_camera_cal.yaml 
```
This script reads a calibration file in yaml/yml format and publishes camera_info messages for a camera, the calibration file is passed as a command line argument. 

### 4. Launch `image_proc` Node

In a new terminal, change the ROS namespace to match the camera output:

```bash
export ROS_NAMESPACE=/iris/usb_cam
```

Launch the `image_proc` node:

```bash
roslaunch image_proc_gazebo.launch
```
It launches `image_proc` node but with camera topics remapped to match the gazebo iris camera topics output name. 

### 5. Continuous AprilTag Detection with `apriltag_ros`

- In a new terminal, Clone the [`apriltag_ros`](https://github.com/AprilRobotics/apriltag_ros) repo and follow the quick start instructions to build it. (Use catkin_make_isolated if you cloned it inside your catkin_ws and catkin_make didn’t work)

- Adjust the ROS namespace:

    ```bash
    export ROS_NAMESPACE=/iris/usb_cam
    ```

- Launch the custom Tag detection node for Gazebo simulation with Iris drone:

    ```bash
    roslaunch gazebo_detection.launch
    ```
  
This launches apriltag_ros continuous detection node but with the camera_name and image_topic remapped to match the topics outputted by gazebo iris camera. 

- Make required modifications in `tags.yaml` and refer to `settings.yaml` under the `config` folder in the `apriltag_ros` directory.

### 6. AprilTag Tracking

To launch mavros along with the apriltag_tracking_node. In a new terminal:

```bash
roslaunch apriltag_tracking.launch
```

## Acknowledgements

- **Dr. Joseph Conroy**: For guidance, resources, and detailed directions.
- **Zach Bortoff**: For his significant contributions to the project.
  
