Ros2 dependencies:
sudo apt update
sudo apt install -y build-essential cmake git

Docker:
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

didnt work:docker pull osrf/ros:humble-desktop
try:docker pull arm64v8/ros:humble-desktop

docker run -it --rm osrf/ros:humble-desktop
