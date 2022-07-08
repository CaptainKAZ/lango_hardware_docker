# sudo docker run -it \
# --name "lango_hardware_container" \
# --network host \
# -v ws:/ros2_ws \
# -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
# -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
# -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg \
# --gpus all \
# -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER \
# lango_hardware &&\
# usermod -u 1000 lango && usermod -g 1000 lango

# --user $(id -u ${USER}):$(id -g ${USER}) -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro -v /etc/shadow:/etc/shadow:ro \

sudo docker run -it \
--privileged \
--net=host \
-v ws:/home/lango/ros2_ws \
-v /run/udev/:/run/udev/ \
-v /dev/bus/usb:/dev/bus/usb \
-v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
-v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY \
-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
-e PULSE_SERVER=$PULSE_SERVER -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
lango_hardware


