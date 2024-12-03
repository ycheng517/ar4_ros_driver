FROM ros:jazzy

WORKDIR /ar4_ws
RUN mkdir -p /ar4_ws/src && \
  cd src && \
  git clone https://github.com/ycheng517/ar4_ros_driver.git

RUN apt update
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
