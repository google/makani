# Creates a debian9 image and runs install_packages.sh.
#
# To create the image, run:
#  $ docker build . -t makani --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g)
#
# To start an instance with the created image and access the terminal, run:
#  $ xhost +
#  $ docker run -it --net=host --privileged -e DISPLAY  \
#      -v /tmp/.X11-unix:/tmp/.X11-unix  \
#      -v /path/to/makani:/home/makani/makani  \
#      makani /bin/bash
#
# To run the sim:
#  $ sudo route add -net 239.0.0.0 netmask 255.0.0.0 dev lo
#  $ run_sim -S -M flight

FROM l.gcr.io/google/debian9:latest

ARG USER_ID
ARG GROUP_ID

RUN apt-get update && apt-get -y install sudo lsb-release wget git procps

RUN addgroup --gid $GROUP_ID user
RUN useradd -m makani -u $USER_ID && echo "makani:makani" | chpasswd && adduser makani sudo

RUN echo "makani ALL=(root) NOPASSWD:ALL" >> /etc/sudoers.d/makani

RUN chmod 0440 /etc/sudoers.d/makani

USER makani
WORKDIR /home/makani

ENV USER=makani

COPY --chown=makani:makani \
    ["lib/scripts/mbash.sh" ,"lib/scripts/system.sh", \
     "/home/makani/makani/lib/scripts/"]
COPY --chown=makani:makani lib/scripts/install/ \
    /home/makani/makani/lib/scripts/install/

ENV MAKANI_HOME=/home/makani/makani

RUN /home/makani/makani/lib/scripts/install/install_packages.sh
