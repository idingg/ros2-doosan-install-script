#!/bin/bash

### change repository to Korea
sudo sed -i "s|$(grep -o 'deb [^ ]*ubuntu/' /etc/apt/sources.list |
grep -m 1 -v security |
sed 's|deb ||g')|http://kr.archive.ubuntu.com/ubuntu/|g" /etc/apt/sources.list

### Delete old packages
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker \
containerd runc; do sudo apt-get purge -y $pkg; done

### Update package list
sudo apt-get update

### Add Docker official GPG key:
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
-o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

### Add docker repository to Apt sources:
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && \
echo "$VERSION_CODENAME") stable" | \
sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

### Install docker
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
docker-buildx-plugin docker-compose-plugin

### Add user to docker group
sudo usermod -aG docker $USER

### Change terminal group to docker if running in script
if [ ! -t 0 ]; then
    group=docker
    if [ $(id -gn) != $group ]; then
    exec sg $group "$0 $*"
    fi
fi

### Install ROS2 humble desktop
### REF: https://github.com/Tiryoh/ros2_setup_scripts_ubuntu
CHOOSE_ROS_DISTRO=humble
INSTALL_PACKAGE=desktop
TARGET_OS=jammy

sudo apt-get update && sudo apt install -y $(sudo apt list --upgradable | 
grep -v Listing... | cut -d "/" -f 1 | grep -v xrdp)

sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt-get install -y curl gnupg2 lsb-release build-essential

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-$CHOOSE_ROS_DISTRO-$INSTALL_PACKAGE
sudo apt-get install -y python3-argcomplete python3-colcon-clean
sudo apt-get install -y python3-colcon-common-extensions
sudo apt-get install -y python3-rosdep python3-vcstool
[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] ||
sudo rosdep init
rosdep update
grep -F "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" ~/.bashrc ||
echo "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" >> ~/.bashrc
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc ||
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
grep -F "export ROS_LOCALHOST_ONLY=1" ~/.bashrc ||
echo "# export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash
# ROS2 Install End

sudo apt-get update
sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget \
ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs \
dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ign-ros2-control

### Install gazebo sim
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \
`lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
### Download gazebo repository key
wget -qO - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -

sudo apt-get update
sudo apt-get install -y libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs \
ros-humble-moveit-msgs ros-humble-ros-gz-sim ros-humble-ros-gz
sudo apt-get autoremove -y

### Doosan ROS2 Packages Installation
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

### Download Doosan ROS2 Packages
sudo apt-get -y install git
git clone -b humble https://github.com/DoosanRobotics/doosan-robot2.git
git clone -b humble https://github.com/ros-controls/gz_ros2_control
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

### Install Doosan robot emulator
cd ~/ros2_ws/src/doosan-robot2/
chmod +x ./install_emulator.sh
sudo ./install_emulator.sh
grep -F 'export PYTHONPATH=$PYTHONPATH:~/ros2_ws/install/dsr_common2/lib/dsr_common2/imp' ~/.bashrc ||
echo 'export PYTHONPATH=$PYTHONPATH:~/ros2_ws/install/dsr_common2/lib/dsr_common2/imp' >> ~/.bashrc

### Add the installation prefix of "ament_cmake"
source /opt/ros/humble/setup.bash
### Build Doosan ROS2 Packages
cd ~/ros2_ws
colcon build
. ~/ros2_ws/install/setup.bash

### Use software rendering if WSL
uname -r | grep -qi "Microsoft" &&
grep -F "export LIBGL_ALWAYS_SOFTWARE=1" ~/.bashrc ||
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc

newgrp docker
