FROM gazebo:gzserver9
FROM gazebo:libgazebo9
FROM ros:melodic

RUN apt-get update 

RUN apt-get update && \
    apt-get install -y ros-melodic-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
      python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt install libboost-all-dev -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get install libboost-all-dev

WORKDIR /root
ENV CATKIN_WS /root/catkin_ws/

RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS
RUN wstool init src

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN bash -c "source /root/.bashrc"
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd $CATKIN_WS; catkin_make'

WORKDIR $CATKIN_WS/src
COPY . .
WORKDIR $CATKIN_WS

RUN apt-get update && apt-get upgrade -q -y && apt-get install libgazebo9-dev -y 
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash'
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin clean -y && catkin build


RUN sed --in-place --expression \
      '$isource "$CATKIN_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

RUN apt-get update && apt-get upgrade -q -y && apt-get install -q -y \
    imagemagick \
    libboost-all-dev \
    libgts-dev \
    libgazebo9-dev \
    libjansson-dev \
    libtinyxml-dev \
    mercurial \
    psmisc\
    && rm -rf /var/lib/apt/lists/*

RUN apt-get install -q -y \
    libgazebo9-dev \
    && rm -rf /var/lib/apt/lists/*

EXPOSE 11345
EXPOSE 11311

CMD ["source devel/setp.bash","roscore"]