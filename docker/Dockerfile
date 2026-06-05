#getting jazzy desktop version, allow for gui stuff
FROM osrf/ros:jazzy-desktop

#setting up no interactive frontend
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
#running these command installing vim,sudo and python3 pip to image, deletes package list once done
RUN apt-get update && apt-get install -y \
    vim \
    sudo \
    bash-completion \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

#Installing pytorch with cuda support and Ultralytics for YOLO
RUN pip3 install --no-cache-dir --break-system-packages \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121 

#ignore installed to help prevent trying to delete numpy version installed in desktop version of ros tht is being imported
RUN pip3 install --no-cache-dir --break-system-packages \
    "numpy<2" \
    ultralytics

#networking stuff to allow robots to talk normal --network=host --ipc=host


#Using entrypoint script to set up enviroment
COPY entrypoint.sh /entrypoint.sh

#switching user value form root to ros from files copied over,copying the files
COPY --chown=ubuntu:ubuntu bashrc /home/ubuntu/.bashrc

#fixing file permission
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

#fixing permission issues for home dic
RUN chown -R ubuntu:ubuntu /home/ubuntu

#ensuring entrypoint.sh can run
RUN chmod +x /entrypoint.sh

#creating the working directory
WORKDIR /DEV_WS
USER ubuntu

ENTRYPOINT ["/bin/bash","/entrypoint.sh"]

CMD ["bash"]
