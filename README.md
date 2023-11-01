# ARL-project

## Setup 

Project is realized in a docker container with ROS2 Foxy on Ubuntu 20.04. 
The `src/` directory contains code for drone's control in ROS from docker's tello_ws ROS workspace directory. 

### Docker container

To setup the docker container go into `docker/` directory and type into the terminal

	`docker compose up` 

### Python 

To install necessary python packages type this into the terminal in your docker container 

	`pip install -r requirements.txt` 



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

