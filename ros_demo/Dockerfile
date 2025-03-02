FROM ros:jazzy

RUN apt-get update && \
    apt-get install -y ros-jazzy-rosbridge-suite ros-jazzy-joy && \
    rm -rf /var/lib/apt/lists/*

CMD ["ros2", "launch", "/root/ros_ws/src/launch_demo.py"]
