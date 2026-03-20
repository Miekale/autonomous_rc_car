xhost +local:docker && docker run -it --rm \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $(pwd):/home \
    --device /dev/video0:/dev/video0 \
    --privileged \
    rc:v1 ./build/perception_demo $1