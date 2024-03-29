# ROS Docker Integration Repository

This repository contains a set of Dockerfiles and a Makefile to facilitate the deployment of ROS (Robot Operating System) Noetic on Ubuntu 20.04 base images, specifically tailored for ARM architectures. It supports the setup for both publishing and receiving data within a ROS network, utilizing Docker containers. Additionally, it includes a script for installing Docker on a Raspberry Pi.

## Dockerfiles

- **Publishing Image (`Dockerfile.publish`)**: Sets up an environment for publishing data to a ROS network. It installs ROS Noetic, the Realsense2 camera package, and the RPLIDAR package. It also configures the environment to initialize `rosdep` and sets bash as the default command.

- **Receiving Image (`Dockerfile.receive`)**: Similar to the publishing image but tailored for receiving data. It also sets up the environment to run `rostopic echo` on a specified RealSense topic by default.

## Makefile Commands

- **Network Management**: Creates a Docker network (`ros-network`) to facilitate communication between containers.
- **Image Building**: Provides commands to build Docker images for both publishing (`build-publish`) and receiving (`build-receive`) data.
- **Container Management**: Includes commands to run and clean up both publishing (`run-publish`, `clean-publish`) and receiving (`run-receive`, `clean-receive`) containers.
- **Image Cleanup**: Offers a command to remove the Docker images (`clean-images`).

## Docker Installation Script

The script `install_docker_on_pi.sh` automates the process of installing Docker on a Raspberry Pi, including adding the Docker repository, installing Docker CE, adding the current user to the Docker group, and enabling Docker to start on boot.

## Usage

To use this repository, clone it to your local machine or Raspberry Pi, ensure Docker is installed and running, and then utilize the Makefile commands to build images, create containers, and manage the network as needed for your ROS Noetic projects.

The recommended `Makefile` order is:

`make create-network`
`make build-publish`
`make run-publish`

## Connecting

Connect onto doug's iphone

The ip address is: 172.20.10.6

ssh ubuntu@172.20.10.6
pass ubuntu
