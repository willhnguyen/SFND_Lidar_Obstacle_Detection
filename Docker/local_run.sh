#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $DIR/..

# Run docker with current directory (pwd) as /workspace
xhost +local:root # for the lazy and reckless
docker run --rm -it \
    --privileged \
    -e DISPLAY=:1 \
    --gpus all \
    --env="QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v `pwd`:/workspace \
    sensor_fusion \
    /bin/bash