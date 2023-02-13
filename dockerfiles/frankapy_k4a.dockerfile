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

# Install Azure-Kinect-Sensor-SDK on Ubuntu 20.04.
# * https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1190
# * https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1263
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
        curl \
        software-properties-common && \
    curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | tee /etc/apt/sources.list.d/microsoft-prod.list && \
    curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \
    apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod || true   && \
    apt-get update || true && \
    echo "libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76" | debconf-set-selections && \
    apt-get install -y libk4a1.4-dev && \
    echo "libk4abt1.1 libk4abt1.1/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38" | debconf-set-selections && \
    apt-get install -y libk4abt1.1-dev && \
    apt-get install -y k4a-tools && \
    add-apt-repository --remove https://packages.microsoft.com/ubuntu/18.04/prod && \
    rm /etc/apt/sources.list.d/microsoft-prod.list && \
    rm /etc/apt/sources.list.d/microsoft-prod.list.save && \
    sed -i "s/1.4/1.4.1/" /usr/lib/cmake/k4abt/k4abtConfig.cmake

# Install Azure_Kinect_ROS_Driver.
RUN apt-get install -y git && \
    cd $HOME/catkin_ws/src && \
    git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver

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
