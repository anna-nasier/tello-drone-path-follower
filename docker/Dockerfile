FROM osrf/ros:foxy-desktop

WORKDIR $HOME
RUN mkdir -p $HOME/tello_ws/src
WORKDIR $HOME/tello_ws/src

RUN apt-get update && apt-get upgrade -y
RUN    	DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
	libasio-dev \
	python3-pip\
	git\
	ros-foxy-cv-bridge\
	ros-foxy-camera-calibration-parsers\
	curl

RUN yes | pip3 install 'transformations==2018.9.5'
RUN curl -sSL http://get.gazebosim.org | sh


RUN git clone https://github.com/clydemcqueen/tello_ros.git
RUN git clone https://github.com/ptrmu/ros2_shared.git

WORKDIR $HOME/tello_ws/

RUN rosdep install -y --from-paths . --ignore-src
RUN echo "source /opt/ros/foxy/setup.bash" >> $HOME/.bashrc

CMD ["/bin/bash"]
