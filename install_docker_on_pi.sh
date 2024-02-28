#!/bin/bash

# Update and upgrade the system packages
sudo apt-get update && sudo apt-get upgrade -y

# Install necessary packages for Docker
sudo apt-get install -y apt-transport-https ca-certificates software-properties-common

# Add Docker’s official GPG key
curl -fsSL https://download.docker.com/linux/raspberry_pi/gpg | sudo apt-key add -

# Add the Docker repository to APT sources
echo "deb [arch=armhf] https://download.docker.com/linux/raspberry_pi $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list

# Update the package database with Docker packages from the newly added repo
sudo apt-get update

# Install Docker
sudo apt-get install -y docker-ce

# Add current user to the Docker group to run Docker commands without sudo
sudo usermod -aG docker $USER

# Print Docker version to verify installation
docker --version

# Enable Docker to start on boot
sudo systemctl enable docker

# Start Docker service
sudo systemctl start docker

echo "Docker installation on Raspberry Pi 5 is complete."