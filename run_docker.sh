#!/bin/bash

IMAGE_NAME="ros2-jazzy-yolo"
CONTAINER_NAME="yolo_ros2_container"
PROJECT_DIR="$(dirname "$(readlink -f "$0")")"


# step 1: allow for gui connections from container to local host
xhost +local:docker > /dev/null

# step 2: check if container with name is running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "container is already running making a new terminal in container"
    docker exec -it $CONTAINER_NAME bash

# step 3: check if container exists but is stopped
elif [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Starting container"
    docker start $CONTAINER_NAME
    echo "accessing container"
    docker exec -it $CONTAINER_NAME bash

# Step 4: if does not exit, build a new one 
else
    echo "Building container with name $CONTAINER_NAME"
    docker run -it \
    --name $CONTAINER_NAME \
    --gpus all \
    --network=host \
    --ipc=host \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PROJECT_DIR/ros2_ws:/DEV_WS/ros2_ws:rw" \
   $IMAGE_NAME 
fi

#cleaning up x11 permission
xhost -local:docker > /dev/null
