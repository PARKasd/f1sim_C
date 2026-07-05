# MIT License

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ros:humble

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       tmux \
                       python3-pip \
                       build-essential \
                       ros-humble-rviz2
RUN pip3 install transforms3d

# this repository provides everything: the gym API, the C++ simulation core,
# and the ROS 2 bridge
RUN mkdir -p /sim_ws/src/f1tenth_gym
COPY . /sim_ws/src/f1tenth_gym

# gym + C++ simulation core
RUN cd /sim_ws/src/f1tenth_gym && \
    pip3 install -e .

# ROS 2 bridge
RUN source /opt/ros/humble/setup.bash && \
    cd /sim_ws && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build --symlink-install --base-paths src/f1tenth_gym/f1tenth_gym_ros

WORKDIR /sim_ws
ENTRYPOINT ["/bin/bash"]
