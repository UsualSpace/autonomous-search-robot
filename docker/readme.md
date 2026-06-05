# how to run the docker command. Requires nvidia gpu for yolo and cuda when it comes done to yolo.
docker run -it \
  --gpus all \
    --ipc=host \
      --network=host \
        --env="DISPLAY" \
          --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            ros2-jazzy-yolo


# UPDATES 
 For different gpu you can check out ultralytics on google.
