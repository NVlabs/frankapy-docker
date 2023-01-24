FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04

# Update apt-get.
RUN apt-get update

# Set the working directory to /root.
WORKDIR /root

# Install ROS Noetic.
RUN apt-get install -y \
        curl \
        lsb-release && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop-full && \
    echo "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc

# Create catkin workspace.
RUN mkdir -p catkin_ws/src

# Install pip.
RUN apt-get install -y python3-pip

# Install frankapy.
RUN apt-get install -y git && \
    pip install -U \
        numpy \
        Pillow \
        protobuf && \
    git clone --recurse-submodules https://github.com/iamlab-cmu/frankapy.git && \
    cd frankapy && \
    pip install -e . && \
    ln -s $HOME/frankapy/catkin_ws/src/franka-interface-msgs $HOME/catkin_ws/src/franka-interface-msgs && \
    apt-get install -y \
        ros-noetic-franka-ros \
        ros-noetic-libfranka

# Build catkin workspace.
RUN cd catkin_ws && \
    /bin/bash -c ". /opt/ros/noetic/setup.bash && \
        catkin_make" && \
    echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc

# Install tmux.
RUN apt-get install -y tmux

# Install rsync.
RUN apt-get install -y rsync

# Install terminator.
RUN apt-get install -y \
    libcanberra-gtk3-module \
    terminator
