FROM ros:humble

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install pytak==6.4.0

# copy rostak package and config 
COPY . rostak_ws/src/rostak
COPY config rostak_ws/src/rostak/config
WORKDIR /rostak_ws

# build rostak package
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# launch rostak
CMD . install/setup.sh && \
    ros2 launch rostak rostak.launch.py
