FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

WORKDIR /ar4_ws
COPY . /ar4_ws/src

RUN apt update
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

COPY ./docker/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
