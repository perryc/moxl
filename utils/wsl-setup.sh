#!/usr/bin/env bash
# ------------------------------------------------------------------
# wsl-setup.sh — Bootstrap ROS2 Jazzy on WSL2 Ubuntu 24.04 (Noble)
#
# Run once from inside WSL:
#   bash /mnt/c/Users/4perr/documents/github/moxl/utils/wsl-setup.sh
#
# What it does:
#   1. Installs ROS2 Jazzy Desktop (includes rviz2, Gazebo, etc.)
#   2. Installs all moxl ROS2 + Python dependencies
#   3. Clones the repo to ~/moxl (native ext4 — fast builds)
#   4. Runs colcon build
#   5. Appends ROS2 sourcing to ~/.bashrc
# ------------------------------------------------------------------

set -euo pipefail

REPO_WIN="/mnt/c/Users/4perr/documents/github/moxl"
REPO_WSL="$HOME/moxl"
ROS_DISTRO="jazzy"

echo "================================================"
echo "  MOXL — WSL2 ROS2 Jazzy Setup"
echo "================================================"
echo ""

# ── 1. Locale ─────────────────────────────────────────────────────
echo "[1/7] Setting locale..."
sudo apt-get update -qq
sudo apt-get install -y -qq locales > /dev/null
sudo locale-gen en_US en_US.UTF-8 > /dev/null
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ── 2. ROS2 apt repository ───────────────────────────────────────
echo "[2/7] Adding ROS2 Jazzy apt repository..."
sudo apt-get install -y -qq software-properties-common curl > /dev/null
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update -qq

# ── 3. Install ROS2 Jazzy Desktop + Gazebo ───────────────────────
echo "[3/7] Installing ROS2 Jazzy Desktop (this takes a few minutes)..."
sudo apt-get install -y -qq \
  ros-${ROS_DISTRO}-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  > /dev/null

# ── 4. Install moxl ROS2 dependencies ────────────────────────────
echo "[4/7] Installing moxl ROS2 package dependencies..."
sudo apt-get install -y -qq \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-diff-drive-controller \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-nmea-navsat-driver \
  ros-${ROS_DISTRO}-nmea-msgs \
  ros-${ROS_DISTRO}-foxglove-bridge \
  ros-${ROS_DISTRO}-foxglove-msgs \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-gz-ros2-control \
  ros-${ROS_DISTRO}-twist-mux \
  ros-${ROS_DISTRO}-teleop-twist-joy \
  ros-${ROS_DISTRO}-joy \
  ros-${ROS_DISTRO}-rqt-robot-steering \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-broadcaster \
  ros-${ROS_DISTRO}-effort-controllers \
  ros-${ROS_DISTRO}-velocity-controllers \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-tf2-geometry-msgs \
  ros-${ROS_DISTRO}-geographic-msgs \
  ros-${ROS_DISTRO}-behaviortree-cpp \
  ros-${ROS_DISTRO}-nlohmann-json-schema-validator-vendor \
  > /dev/null

# ── 5. Python dependencies ───────────────────────────────────────
echo "[5/7] Installing Python dependencies..."
sudo apt-get install -y -qq python3-pip python3-pyproj python3-shapely python3-numpy > /dev/null

# ── 6. Clone / sync repo to native Linux filesystem ─────────────
echo "[6/7] Syncing repo to native filesystem at ${REPO_WSL}..."
if [ -d "${REPO_WSL}/.git" ]; then
  echo "  Repo already exists — pulling latest..."
  cd "${REPO_WSL}"
  git pull --ff-only || echo "  (pull skipped — may have local changes)"
else
  # Clone from the Windows mount so we keep the same remote
  REMOTE_URL=$(git -C "${REPO_WIN}" remote get-url origin 2>/dev/null || echo "")
  if [ -n "${REMOTE_URL}" ]; then
    git clone "${REMOTE_URL}" "${REPO_WSL}"
  else
    # No remote — just copy the tree
    echo "  No git remote found — copying working tree..."
    mkdir -p "${REPO_WSL}"
    rsync -a --exclude=build --exclude=install --exclude=log \
      "${REPO_WIN}/" "${REPO_WSL}/"
  fi
fi
cd "${REPO_WSL}"

# ── 7. Initialize rosdep & build ─────────────────────────────────
echo "[7/7] Building workspace..."
source /opt/ros/${ROS_DISTRO}/setup.bash

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init 2>/dev/null || true
fi
rosdep update --rosdistro=${ROS_DISTRO} 2>/dev/null

# Install any remaining deps rosdep can find
rosdep install --from-paths . --ignore-src -y -r 2>/dev/null || true

colcon build --symlink-install --base-paths . src/toolpath_planner
echo ""
echo "Build complete."

# ── Shell config ─────────────────────────────────────────────────
BASHRC_MARKER="# >>> moxl ros2 >>>"
if ! grep -q "${BASHRC_MARKER}" ~/.bashrc 2>/dev/null; then
  echo ""
  echo "Adding ROS2 + moxl workspace sourcing to ~/.bashrc..."
  cat >> ~/.bashrc << 'BASHRC'

# >>> moxl ros2 >>>
source /opt/ros/jazzy/setup.bash
if [ -f ~/moxl/install/setup.bash ]; then
  source ~/moxl/install/setup.bash
fi
export ROS_DOMAIN_ID=0
export MOXL_WS=~/moxl
# <<< moxl ros2 <<<
BASHRC
fi

echo ""
echo "================================================"
echo "  Setup complete!"
echo ""
echo "  Workspace:  ${REPO_WSL}"
echo "  ROS distro: ${ROS_DISTRO}"
echo ""
echo "  Quick start:"
echo "    wsl -d Ubuntu-24.04"
echo "    cd ~/moxl"
echo "    make sim       # launch Gazebo simulation"
echo "    make test      # run unit tests"
echo ""
echo "  From Windows terminal:"
echo "    make wsl-sim   # one-liner (from repo root)"
echo "================================================"
