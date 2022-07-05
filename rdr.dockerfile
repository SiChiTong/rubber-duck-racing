FROM dustynv/ros:foxy-pytorch-l4t-r34.1.1
RUN sudo apt-get update
RUN sudo apt-get upgrade
RUN sudo apt-get install nano libi2c-dev i2c-tools
RUN git clone https://github.com/alex-k-exe/rubber-duck-racing
RUN cd rubber-duck-racing/dev_ws
RUN git clone https://github.com/ros-perception/vision_opencv
RUN cd vision_opencv
RUN git checkout ros2
RUN cd ..
RUN colcon build --symlink-install
RUN . install/setup.bash
RUN echo "All done!!!!"