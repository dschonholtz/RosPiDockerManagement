# Define the image names
IMAGE_NAME_RECEIVE=ros-noetic-receive
IMAGE_NAME_PUBLISH=ros-noetic-publish
IMAGE_NAME_MASTER=ros-noetic-master

# Define the container names
CONTAINER_NAME_RECEIVE=ros-receive-container
CONTAINER_NAME_PUBLISH=ros-publish-container
CONTAINER_NAME_MASTER=ros-master-container

ROS_MASTER_URI=http://192.168.1.45:11311
ROS_IP=192.168.1.45
LAUNCH_VOLUME=/home/ubuntu/catkin_ws/
SCRIPTS_VOLUME=./scripts_volume


# Build the Docker image for the ROS master node using Dockerfile.roscore
build-master:
	docker build -f Dockerfile.roscore -t $(IMAGE_NAME_MASTER) .


# Build the Docker image for receiving data using Dockerfile.receive
build-receive:
	docker build -f Dockerfile.receive -t $(IMAGE_NAME_RECEIVE) .

run-master: create-network
	docker run -d --rm --network $(NETWORK_NAME) \
	--name $(CONTAINER_NAME_MASTER) \
	$(IMAGE_NAME_MASTER)

# Build the Docker image for publishing data using Dockerfile.publish
build-publish:
	docker build -f Dockerfile.publish -t $(IMAGE_NAME_PUBLISH) .

# Run the Docker container for publishing data and act as ROS master
# This currently does not work, but the command is useful
run-publish:
	docker run -it --rm --network host \
	--name $(CONTAINER_NAME_PUBLISH) \
	-e ROS_MASTER_URI=$(ROS_MASTER_URI) \
	-e ROS_IP=$(ROS_IP) \
	-v $(LAUNCH_VOLUME):/home/ubuntu/catkin_ws \
	-v $(SCRIPTS_VOLUME):/home/ubuntu/scripts \
	--privileged \
	$(IMAGE_NAME_PUBLISH)


publish-interactive:
	docker run -it --rm --network host \
	--name $(CONTAINER_NAME_PUBLISH) \
	-e ROS_MASTER_URI=$(ROS_MASTER_URI) \
	-e ROS_IP=$(ROS_IP) \
	-v $(LAUNCH_VOLUME):/home/ubuntu/catkin_ws \
	-v $(SCRIPTS_VOLUME):/home/ubuntu/scripts \
	--privileged \
	$(IMAGE_NAME_PUBLISH) /bin/bash


# Run the Docker container for receiving data
run-receive: create-network
	docker run -it --rm --network $(NETWORK_NAME) \
	--name $(CONTAINER_NAME_RECEIVE) \
	-e ROS_MASTER_URI=http://$(CONTAINER_NAME_MASTER):11311 \
	$(IMAGE_NAME_RECEIVE)

# Stop and remove the master container
clean-master:
	docker stop $(CONTAINER_NAME_MASTER)
	docker rm $(CONTAINER_NAME_MASTER)


# Stop and remove the receive container
clean-receive:
	docker stop $(CONTAINER_NAME_RECEIVE)
	docker rm $(CONTAINER_NAME_RECEIVE)

# Stop and remove the publish container
clean-publish:
	docker stop $(CONTAINER_NAME_PUBLISH)
	docker rm $(CONTAINER_NAME_PUBLISH)

# Remove the Docker images
clean-images:
	docker rmi $(IMAGE_NAME_RECEIVE)
	docker rmi $(IMAGE_NAME_PUBLISH)
	docker rmi $(IMAGE_NAME_MASTER)
