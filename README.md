# FrankaPy Docker

Running **FrankaPy** workflow all in **Docker**.

### Contents

1. [Requirements](#requirements)
2. [Installation](#installation)
    1. [Control PC](#control-pc)
    2. [FrankaPy PC](#frankapy-pc)
3. [Running the Robot](#running-the-robot)
4. [Appendices](#appendices)
    1. [Installing docker](#installing-docker)
    2. [Installing nvidia-docker](#installing-nvidia-docker)

## Requirements

- A computer with Ubuntu Realtime Kernel - referred to as the **Control PC** below.
- A computer with Ubuntu - referred to as the **FrankaPy PC** below.
- Set up the network of Control PC and Franka Control Interface (FCI) following the [instructions](https://iamlab-cmu.github.io/franka-interface/network.html) from `franka-interface`. See [instructions](https://frankaemika.github.io/docs/getting_started.html) from FCI for more information.

## Installation

Run the installation steps below correspondingly for the [Control PC](#control-pc) and [FrankaPy PC](#frankapy-pc).

### Control PC

1. Install `docker`. See the [Installing docker](#installing-docker) section.

2. Clone the repo with `--recursive` and and cd into it:

    ```Shell
    git clone --recursive ssh://git@gitlab-master.nvidia.com:12051/ychao/frankapy-docker.git
    cd frankapy-docker
    ```

3. Build a docker image for `franka-interface`:

    ```Shell
    ./scripts/build_franka-interface.sh
    ```

### FrankaPy PC

1. Install `docker`. See the [Installing docker](#installing-docker) section.

2. Install `nvidia-docker`. See the [Installing nvidia-docker](#installing-nvidia-docker) section.

3. Clone the repo with `--recursive` and and cd into it:

    ```Shell
    git clone --recursive ssh://git@gitlab-master.nvidia.com:12051/ychao/frankapy-docker.git
    cd frankapy-docker
    ```

4. Build a docker image for `frankapy`:

    ```Shell
    ./scripts/build_frankapy.sh
    ```

    Alternatively, if you also need the Azure Kinect, we also provide a build script for that:

    ```Shell
    ./scripts/build_frankapy_k4a.sh
    ```

## Running the Robot

1. Unlock the robot following the [instructions](https://iamlab-cmu.github.io/frankapy/running.html#unlocking-the-franka-robot) from `FrankaPy`. Make sure to unlock in the web interface and also release the activation device.

2. Connect to the FrankaPy PC via VNC or SSH, and cd to the `frankapy-docker/` folder.

3. Connect to the Control PC via VNC or SSH, and cd to the `frankapy-docker/` folder.

4. On the **FrankaPy PC**, run a docker container for `frankapy` in interactive mode:

    ```Shell
    ./scripts/run_frankapy.sh
    ```

    or if you built the image with Azure Kinect, run:

    ```Shell
    ./scripts/run_frankapy_k4a.sh
    ```

    Both commands will start a new `terminator` window (or `tmux` session) and launch `roscore`.

    - We use `terminator` and `tmux` for launching processes in multiple terminals at once. The script will decide which one to use based on the availability of display. You can also manually set the choice by setting an environment variable `CONSOLE`, for example:

        ```Shell
        CONSOLE=tmux ./scripts/run_frankapy.sh
        ```

        or

        ```Shell
        CONSOLE=terminator ./scripts/run_frankapy.sh
        ```

    - While we are in the container, by default we will just have access to the `frankapy` repo copied during docker build. In practice, we may want to run with an updated `frankapy` repo as we develop on the fly. One way to achieve that is to mount a `frankapy` repo from your host into the container, and overwrite the default `frankapy` repo created during docker build. This has been set up and can be simply done by setting an environment variable `FRANKAPY_DIR`:

        ```Shell
        FRANKAPY_DIR=/path/to/frankapy/on/host ./scripts/run_frankapy.sh
        ```

        This way you can run with a dynamic `frankapy` repo and do not need to re-build the docker image after making changes.

5. On the **Control PC**, run a docker container for `franka-interface` in interactive mode:

    ```Shell
    ROS_MASTER_URI=$YOUR_ROS_MASTER_URI ./scripts/run_franka-interface.sh
    ```

    Note that `$ROS_MASTER_URI` needs to be set above according to the IP or hostname of the FrankaPy PC.

    This will launch `franka_interface`, `franka_ros_interface`, and `franka_gripper` in three separate terminals on the Control PC.

    - Likewise, the script will choose between `terminator` and `tmux` based on the availability of display. You can also manually set the choice by setting an environment variable `CONSOLE`, for example:

        ```Shell
        CONSOLE=tmux ROS_MASTER_URI=$YOUR_ROS_MASTER_URI ./scripts/run_franka-interface.sh
        ```

        or

        ```Shell
        CONSOLE=terminator ROS_MASTER_URI=$YOUR_ROS_MASTER_URI ./scripts/run_franka-interface.sh
        ```

6. Finally, go back to the **FrankaPy PC**. In the latest opened window in `tmux` or `terminator`, you should be able to run an example from `frankapy` now, e.g.,:

    ```Shell
    python3 frankapy/examples/example_movements.py
    ```

## Appendices

### Installing docker

Install and set up `docker`:

```Shell
# Set up the repository.
sudo apt-get update
sudo apt-get install \
  curl
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine.
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Verify that Docker Engine is installed correctly by running the hello-world image.
sudo docker run hello-world

# To manage Docker as a non-root user, add your user to the docker group. Replace $USERNAME with your username.
sudo usermod -aG docker $USERNAME
```

Once you add your user to the docker group, you need to **log out and log back in** to make the effect take place.

See the official docs [here](https://docs.docker.com/engine/install/ubuntu/) and [here](https://docs.docker.com/engine/install/linux-postinstall/) for more information.

### Installing nvidia-docker

1. Install NVIDIA driver. The recommended way is by installing the CUDA Toolkit. For installing CUDA Toolkit 11.7.0 on Ubuntu 20.04, use the following commands:

    ```Shell
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
    sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
    mkdir -p $HOME/admin/deb_files && cd $HOME/admin/deb_files
    wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb
    sudo dpkg -i cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb
    sudo cp /var/cuda-repo-ubuntu2004-11-7-local/cuda-*-keyring.gpg /usr/share/keyrings/
    sudo apt-get update
    sudo apt-get -y install cuda

    # Add to $HOME/.bashrc:
    #   export PATH=/usr/local/cuda-11.7/bin${PATH:+:${PATH}}
    ```

    See the [official download site](https://developer.nvidia.com/cuda-downloads) for more information.

    After rebooting, you should be able to run `nvidia-smi` and see the version of your NVIDIA driver:

    ```Shell
    +-----------------------------------------------------------------------------+
    | NVIDIA-SMI 515.43.04    Driver Version: 515.43.04    CUDA Version: 11.7     |
    |-------------------------------+----------------------+----------------------+
    ```

2. Install and set up `nvidia-docker`:

    ```Shell
    # Setup the package repository and the GPG key.
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
          && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
          && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
                sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
                sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

    # Install the nvidia-container-toolkit package and dependencies.
    sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

    # Configure the Docker daemon to recognize the NVIDIA Container Runtime.
    sudo nvidia-ctk runtime configure --runtime=docker

    # Restart the Docker daemon to complete the installation.
    sudo systemctl restart docker

    # Test run a base CUDA container.
    docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi
    docker image rm nvidia/cuda:11.6.2-base-ubuntu20.04 -f
    ```

    See the official docs [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian) for more information.
