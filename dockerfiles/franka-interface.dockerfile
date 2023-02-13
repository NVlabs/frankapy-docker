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

# Copy franka-interface into container.
COPY franka-interface franka-interface

# Install franka-interface.
RUN apt-get install -y \
        git \
        wget && \
    cd franka-interface && \
    git clone https://github.com/frankaemika/libfranka.git && \
    cd libfranka && \
    git checkout f3b8d77 && \
    git submodule update --init --recursive && \
    cd .. && \
    bash ./bash_scripts/make_libfranka.sh && \
    bash ./bash_scripts/make_franka_interface.sh && \
    sh \
        -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
            > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y python3-catkin-tools && \
    /bin/bash -c ". /opt/ros/noetic/setup.bash && \
        bash ./bash_scripts/make_catkin.sh" && \
    echo "source $HOME/franka-interface/catkin_ws/devel/setup.bash" >> $HOME/.bashrc && \
    apt-get install -y \
        ros-noetic-franka-ros \
        ros-noetic-libfranka

# Install tmux.
RUN apt-get install -y tmux

# Install rsync.
RUN apt-get install -y rsync

# Install terminator.
RUN apt-get install -y \
        libcanberra-gtk3-module \
        terminator
