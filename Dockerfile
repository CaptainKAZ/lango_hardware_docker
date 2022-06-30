ARG ROS_DISTRO="foxy"
FROM ros:${ROS_DISTRO} AS deploy

# Change source and make sure everything is up to date
RUN sed -i 's/archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list &&\
    sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list &&\
    apt-get update && \
    apt-get -y upgrade && \
    rm -rf /var/lib/apt/lists/*

# ROS2 Control, Python3 ,Pip3, Odrive Tool and libusb environment
# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-qpoases-vendor ros-${ROS_DISTRO}-xacro \
    libusb-1.0-0-dev python3 python3-pip wget can-utils && \
    pip3 install odrive==0.5.1.post0 && \
    rm -rf /var/lib/apt/lists/* && sudo rm -r ~/.cache/pip

# Install Pinocchio (with Eigen 3)
RUN apt-get update && \
    apt-get install -qqy lsb-release gnupg2 curl libeigen3-dev && \
    echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add - && \
    apt-get update && apt-get install -qqy robotpkg-py38-pinocchio && \
    rm -rf /var/lib/apt/lists/*
ARG PATH=/opt/openrobots/bin:$PATH \
    PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH \
    LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH  \
    PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH \
    CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH\
    PYTHONUNBUFFERED=1

# Install Tiny Spline
RUN wget https://github.com/msteinbeck/tinyspline/releases/download/v0.4.0/tinyspline-0.4.0-Linux.deb && \
    dpkg -i tinyspline-0.4.0-Linux.deb && \
    rm  tinyspline-0.4.0-Linux.deb

# Install osqp, osqp-eigen
RUN cd / && \
    git clone --recursive https://github.com/osqp/osqp && cd osqp && mkdir build && cd build && cmake -G "Unix Makefiles" .. && cmake --build . --target install && \
    rm -rf /osqp/ && cd / && \
    git clone https://github.com/robotology/osqp-eigen.git && cd osqp-eigen && mkdir build && cd build && cmake .. && make && make install && \
    rm -rf /osqp-eigen && cd / 



# Change root bashrc
RUN echo "source /opt/ros/foxy/setup.bash"  >> /root/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc && \
    echo "export PATH=/opt/openrobots/bin:\$PATH" >> /root/.bashrc && \
    echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH">> /root/.bashrc && \
    echo "export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH" >> /root/.bashrc && \
    echo "export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH" >> /root/.bashrc

RUN sed --in-place --expression \
      '$isource "/home/lango/ros2_ws/install/setup.bash"' \
      /ros_entrypoint.sh

# Create user
RUN useradd --no-log-init --create-home --shell /bin/bash lango &&\
    echo 'lango:ares' | chpasswd &&\
    adduser lango sudo

USER lango

# Change lango bashrc
RUN echo "source /opt/ros/foxy/setup.bash"  >> ~/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "export PATH=/opt/openrobots/bin:\$PATH" >> ~/.bashrc && \
    echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH">> ~/.bashrc && \
    echo "export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH" >> ~/.bashrc && \
    echo "export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH" >> ~/.bashrc

# Clone and build odrive_ros2_control, quadruped_ros2_control (without building gazebo plugin)
# As it uses huge amount of memory(at least 10G) and always failed to build (OOM), seperate it as a new layer. You may need to reduce the num workers.
RUN mkdir -p ~/ros2_ws/src && \
    cd ~/ros2_ws/src && \
    git clone -b fw-v0.5.1 https://github.com/CaptainKAZ/odrive_ros2_control.git && \
    git clone https://github.com/CaptainKAZ/quadruped_ros2_control.git && \
    . /opt/ros/foxy/setup.sh && \
    cd ~/ros2_ws && colcon build --symlink-install --cmake-args -D CMAKE_BUILD_TYPE=Release\
    --packages-skip quadruped_controllers quadruped_gazebo && \
    colcon build --symlink-install --cmake-args -D CMAKE_BUILD_TYPE=Release --parallel-workers 4 \
    --packages-select quadruped_controllers --event-handlers console_direct+
WORKDIR /home/lango/ros2_ws

FROM deploy AS development

USER root
# Install Gazebo Simulation
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' &&\
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - &&\
    apt-get update &&\
    apt-get install -y gazebo11 ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros2-control ros-foxy-desktop mesa-utils 

USER lango
# Build the gazebo plugin
RUN . /opt/ros/foxy/setup.sh && \
    cd ~/ros2_ws && colcon build --symlink-install --cmake-args -D CMAKE_BUILD_TYPE=Release\
    --packages-select quadruped_gazebo

RUN mkdir -p ~/.gazebo/models && cd ~/.gazebo/models && wget http://file.ncnynl.com/ros/gazebo_models.txt &&\
    wget -i gazebo_models.txt && ls model.tar.g* | xargs -n1 tar xzvf

ARG DEBIAN_FRONTEND=noninteractive

ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
RUN echo "export LD_LIBRARY_PATH=/usr/lib/wsl/lib:\$LD_LIBRARY_PATH">> ~/.bashrc 