# ARL-project
 The goal of this project is to develop a drone control system that enables precise and autonomous navigation along a predefined trajectory drawn on a physical piece of paper. Using computer vision and ROS, the system will interpret the trajectory markings which are assumed to be red, green and blue lines on a field marked with black outline, allowing the drone to follow the designated path within a defined coordinate system. This solution aims to bridge the gap between manual input and automated drone navigation, showcasing the potential for intuitive and user-friendly control interfaces in the field of unmanned aerial vehicles.
## Enviroment setup 

In the physical environment, we use DJJ Ryze Tello drone with motion capture system Optitrack in PUT Poznań laboratory and a computer with code commited on this repo. 
The computer is connected to the drone through it's WIFI and Optitrack coordinates are sent to ROS navigation package by ethernet cable and a DHCP protocol. 

For the virtual enviroment and simulation, we use a docker container with Gazebo and ROS2 Foxy on Ubuntu 20.04. Dockerfile and docker-compose files are available in the `docker` folder. 

The `src/` directory contains our code for drone's navigation and machine vision as well as a tello_ros package from https://github.com/clydemcqueen/tello_ros used as a driver, communication and simulation purposes. 

### Docker container

To setup the docker container go into `docker/` directory and type into the terminal

	`docker compose up` 

### Python 

To install necessary python packages type this into the terminal in your docker container 

	`pip install -r requirements.txt` 

## Scope of work 

Intended scope of work is to implement
* a **navigation system** based on potential vector fields with position and orientation feedback from Optitrack 
* a **machine vision system** that 
  * will recognise a piece of paper and the flying zone marked with black outline 
  * recognise the trajectory lines, where the colors of the line are indicators for the z axis 
  * convert the lines to 3D trajectory points within the physical bounds of the flying zone in the lab  
  * send them to navigation

Navigation realised by Maja Zelmanowska. 
Machine vision realised by Anna Nasierowska. We both helped each other and had to integrate our scripts, so it's not completely separate work. 

## Realization 

We realised most of the intended workload, but for simple cases and some initial conditions required. 

First of all, all the code is for ROS2. We communicate with drone through ROS2 nodes in two packages - navigation and camsub. 

**Camsub** is a python package with a subscriber that listens to the camera image from the drone, which is under the `image_raw` topic. It takes each frame and checks for the piece of paper with the flying zone marked using opencv and opencv bridge for converting the image to a numpy array. 

### Finding the paper 

Finding the paper in the image is done by extracting edges from the frame with Canny algorithm, finding their contours and approximating the best matching rectangle with cv2.PolyDP. 

### Lines and trajectory

For the navigation system we used, what is needed to be obtained from the picture is a list of points for the drone to achieve. For the trajectory to be accurate, we need to know each point in space where the direction of movement changes in any axis, so the minimum is a point where each line begins and ends and a color of each line. 

For that purpose, the algoritm of Shi-Tomasi for corner detection is used. It's a good choice, because it is based on finding a siginificant change in each image direction and returning points, which is exactly what we want. Also after testing out a few options, we find it more robust to image noise and easier to parametrize for our usecase. It's more effective in comparison to other feature extractions like edge detection, finding lines using Hough detector, morphologies or color filtering. 

The algorithm is implemented in opencv as `cv2.goodFeaturesToTrack()` function and can be adjusted by giving a maximum amount of points to detect, required quality of points and a minimum distance between them. 

However obtaining the points isn't enough for a trajectory. The coordinates must be in a proper order and of known color. 
That's where the initial conditions and simplification comes in. 
For simplification, the initial condition is that a starting point is the one that is closest to center and the distance from center is what the points are sorted by. 

For color recognition, the value of the hue channel is thresholded in HSV in the corner pixel. Another simplification is that only 3 colors are considered, red, green and blue. A simple function with ifs that assigns 0.5, 1.0 and 1.5m depending on the color. 

### Using the trajectory for navigation 

The points are published in 3 lists - one for each axis. 

## Navigation 


### Credits

The packages 

* `tello_driver`
* `tello_msgs`
* `tello_description`
* `tello_gazebo` 

are from 

authors:
- family-names: "McQueen"
  given-names: "Clyde"
  orcid: "https://orcid.org/0009-0007-9179-318X"
authors:
- family-names: "Mullen"
  given-names: "Peter"
  orcid: "https://orcid.org/0009-0000-7266-8913"
title: "Tello ROS"
version: 0.1.0
date-released: 2019-1-19
url: "https://github.com/clydemcqueen/tello_ros"

