xhost +local:root

docker run -it --privileged --name=unitree_sdk2_container --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume ./unitree_sdk2/example/:/home/user/unitree_sdk2/example/ --volume ./cufino_ws_h1_2/src:/home/user/cufino_ws_h1_2/src unitree_sdk2
