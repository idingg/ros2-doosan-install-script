#!/bin/bash
### Set repository connetion timeout
echo -e 'Acquire::http::Timeout "20";\nAcquire::ftp::Timeout "20";'|sudo tee /etc/apt/apt.conf.d/99timeout

### change permission current shell script file
chmod +x $(realpath $0)

### change repository to Korea
sudo sed -i "s|$(grep -o 'deb [^ ]*ubuntu/' /etc/apt/sources.list | grep -m 1 -v security | sed 's|deb ||g')|http://kr.archive.ubuntu.com/ubuntu/|g" /etc/apt/sources.list
### Delete a line blow if repository connection error
sudo sed -i "s|$(grep -o 'deb [^ ]*ubuntu/' /etc/apt/sources.list | grep -m 1 -v security | sed 's|deb ||g')|https://mirror.techlabs.co.kr/ubuntu/|g" /etc/apt/sources.list

### Delete old packages
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get purge -y $pkg; done

### Update package list
sudo apt-get update

### Add Docker official GPG key:
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

### Add the repository to Apt sources:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" |  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

### Install docker
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

### Add user to docker group
sudo usermod -aG docker $USER

### Change terminal group in script
group=docker
if [ $(id -gn) != $group ]; then
  exec sg $group "$0 $*"
fi

### Install ros-humble-desktop
cd /tmp
wget -T 1 -N https://raw.githubusercontent.com/Tiryoh/ros2_setup_scripts_ubuntu/refs/heads/main/ros2-humble-desktop-main.sh
chmod +x ros2-humble-desktop-main.sh
./ros2-humble-desktop-main.sh

sudo apt-get update
sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ign-ros2-control

### Install gazebo sim
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
### Download gazebo repository key
wget -qO - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -

sudo apt-get update
sudo apt-get install -y libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs ros-humble-ros-gz-sim ros-humble-ros-gz
sudo apt-get autoremove -y

### Package Installation
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

### Download doosan-robot2
sudo apt-get -y install git
git clone -b humble https://github.com/DoosanRobotics/doosan-robot2.git
git clone -b humble https://github.com/ros-controls/gz_ros2_control
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

### Install emulator
cd ~/ros2_ws/src/doosan-robot2/
chmod +x ./install_emulator.sh
./install_emulator.sh

echo 'export PYTHONPATH=$PYTHONPATH:~/ros2_ws/install/dsr_common2/lib/dsr_common2/imp' >> ~/.bashrc

### Add the installation prefix of "ament_cmake"
source /opt/ros/humble/setup.bash

### Build doosan ros2
cd ~/ros2_ws
colcon build
. install/setup.bash

### Use software rendering if WSL
uname -r | grep -qi "Microsoft" && echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc

newgrp docker
