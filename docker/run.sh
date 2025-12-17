xhost +local:root

docker run -it --rm --privileged --runtime=nvidia --gpus all \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="XAUTHORITY=$XAUTH" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--network=host \
--ipc=host \
--shm-size=2gb \
nidhi/px4_1.16_ros2
