FROM ros:iron

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt update && apt install -y git python3-pip

RUN git clone https://github.com/ycheng517/ar4_ros_driver.git

RUN rosdep update

RUN rosdep install --from-paths ar4_ros_driver --ignore-src -r -y

# For calibration
RUN apt-get install -y \
  ros-${ROS_DISTRO}-librealsense2* \
  ros-${ROS_DISTRO}-realsense2-*
RUN apt install ros-${ROS_DISTRO}-rqt-py-common
