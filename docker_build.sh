#!/bin/bash

docker build . -t makani \
    --build-arg USER_ID=$(id -u) \
    --build-arg GROUP_ID=$(id -g)