#!/usr/bin/env bash
# Week 0 (ueb0.pdf): ROS 2 Jazzy on Ubuntu 24.04 in WSL, disk and workspace on drive E.
# Run as root:  wsl -d Ubuntu-24.04 -u root -- bash /mnt/e/aufgaben-public/setup_week0.sh
set -euo pipefail

ROS_USER=ubuntu
WS=/mnt/e/aufgaben-public/ros2_ws
export DEBIAN_FRONTEND=noninteractive

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root." >&2
  exit 1
fi
if ! grep -q 'VERSION_ID="24.04"' /etc/os-release; then
  echo "This script expects Ubuntu 24.04." >&2
  exit 1
fi

echo "== locale"
apt-get update
apt-get install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "== universe and ROS 2 apt source (docs.ros.org Jazzy, Ubuntu debs)"
apt-get install -y software-properties-common curl ca-certificates gnupg
add-apt-repository -y universe
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
dpkg -i /tmp/ros2-apt-source.deb

echo "== marina apt source (stelzo.codeberg.page/marina/installation/packages.html)"
curl -fsSL "https://ppa.steado.tech/ubuntu/key.gpg" | gpg --dearmor --yes \
  -o /usr/share/keyrings/steado-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/steado-archive-keyring.gpg] https://ppa.steado.tech/ubuntu noble main" \
  > /etc/apt/sources.list.d/steado.list

echo "== packages"
apt-get update
apt-get upgrade -y
apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  ros-jazzy-example-interfaces \
  ros-jazzy-rqt-tf-tree \
  ros-jazzy-rosbag2-storage-mcap \
  marina \
  git

echo "== rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
sudo -u "${ROS_USER}" -H rosdep update

echo "== default WSL user"
if ! grep -q '^\[user\]' /etc/wsl.conf; then
  printf '\n[user]\ndefault=%s\n' "${ROS_USER}" >> /etc/wsl.conf
fi

echo "== .bashrc"
BASHRC="/home/${ROS_USER}/.bashrc"
if ! grep -q 'Mobile Roboter, ROS 2 Jazzy' "${BASHRC}"; then
  cat >> "${BASHRC}" <<EOF

# Mobile Roboter, ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
  source "${WS}/install/setup.bash"
fi
EOF
fi

echo "== done: ROS 2 Jazzy installed"
