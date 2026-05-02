FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        ros-humble-turtlebot3-description \
        ros-humble-turtlebot3-msgs \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-plugins \
        libgazebo-dev \
        nano \
        less \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
        torch==2.2.2 \
        torchvision==0.17.2 \
        --index-url https://download.pytorch.org/whl/cu121 \
 && pip3 install --no-cache-dir 'numpy<2' matplotlib pandas 'pyqtgraph<0.13' PyQt5

RUN { \
        echo 'source /opt/ros/humble/setup.bash'; \
        echo 'export TURTLEBOT3_MODEL=burger'; \
        echo 'export DRLNAV_BASE_PATH=/workspace'; \
        echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi'; \
        echo 'source /usr/share/gazebo/setup.sh'; \
        echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspace/src/turtlebot3_simulations/turtlebot3_gazebo/models'; \
        echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/workspace/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/obstacle_plugin/lib'; \
    } >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]