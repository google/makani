#!/bin/bash

xhost +
$ docker run -it --net=host --privileged -e DISPLAY  \
    -v /tmp/.X11-unix:/tmp/.X11-unix  \
    -v ${PWD}:/home/makani/makani  \
    makani /bin/bash