FROM ros:iron

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt update && apt install -y git python3-pip

RUN git clone https://github.com/ycheng517/ar4_ros_driver.git

WORKDIR /ar4_ros_driver

RUN vcs import . --input ar_hand_eye/hand_eye_calibration.repos

RUN rosdep update

RUN rosdep install --from-paths . --ignore-src -r -y
