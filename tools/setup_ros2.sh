#!/usr/bin/env bash
# =============================================================================
# setup_ros2.sh - install the dependencies of limo_mocap_suite with apt and
#                 build it, on ROS 2 Foxy (Ubuntu 20.04, Python 3.8) or
#                 Humble (Ubuntu 22.04, Python 3.10).
#
# Usage (from the workspace root, i.e. the folder that contains src/):
#   ./tools/setup_ros2.sh            check, show what will be installed, ask, install, build
#   ./tools/setup_ros2.sh --check    only run tools/check_deps.py (installs nothing)
#   ./tools/setup_ros2.sh -y         do not ask before installing
#   ./tools/setup_ros2.sh --no-build install only
#
# What it does NOT do: it never installs another Python or changes
# /usr/bin/python3. ROS 2 binaries are compiled for the distro's system Python
# (Foxy -> 3.8, Humble -> 3.10); a different Python cannot import rclpy.
# =============================================================================
set -euo pipefail

YES=0; BUILD=1; CHECK_ONLY=0
for a in "$@"; do
  case "$a" in
    -y|--yes) YES=1 ;;
    --no-build) BUILD=0 ;;
    --check) CHECK_ONLY=1 ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "Unknown option: $a"; exit 2 ;;
  esac
done

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

# ---------------------------------------------------------------- detect distro
. /etc/os-release
case "${VERSION_ID}" in
  20.04) DISTRO=foxy ;;
  22.04) DISTRO=humble ;;
  *) echo "Ubuntu ${VERSION_ID} is not supported (use 20.04 + Foxy or 22.04 + Humble)."; exit 1 ;;
esac
if [ -n "${ROS_DISTRO:-}" ] && [ "${ROS_DISTRO}" != "${DISTRO}" ]; then
  echo "Note: ROS_DISTRO=${ROS_DISTRO} is sourced but Ubuntu ${VERSION_ID} pairs with ${DISTRO}."
  DISTRO="${ROS_DISTRO}"
fi
if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "/opt/ros/${DISTRO} not found. Install ROS 2 ${DISTRO} first (ros-${DISTRO}-ros-base)."
  exit 1
fi
# shellcheck disable=SC1090
set +u; source "/opt/ros/${DISTRO}/setup.bash"; set -u
echo "Ubuntu ${VERSION_ID}, ROS 2 ${DISTRO}, $(/usr/bin/python3 --version)"

# ---------------------------------------------------------------- check first
echo; echo ">>> Dependency check (before)"
/usr/bin/python3 tools/check_deps.py || true
[ "$CHECK_ONLY" = 1 ] && exit 0

# ---------------------------------------------------------------- install
PKGS=(
  python3-numpy python3-scipy python3-matplotlib python3-tk python3-yaml python3-setuptools
  python3-colcon-common-extensions
  "ros-${DISTRO}-rclpy" "ros-${DISTRO}-nav-msgs" "ros-${DISTRO}-geometry-msgs"
  "ros-${DISTRO}-sensor-msgs" "ros-${DISTRO}-std-msgs"
  "ros-${DISTRO}-launch" "ros-${DISTRO}-launch-ros" "ros-${DISTRO}-ros2launch"
  "ros-${DISTRO}-ament-index-python"
)
MISSING=()
for p in "${PKGS[@]}"; do
  dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p")
done
if [ ${#MISSING[@]} -eq 0 ]; then
  echo; echo ">>> All apt packages already installed."
else
  echo; echo ">>> Will run: sudo apt install ${MISSING[*]}"
  if [ "$YES" != 1 ]; then
    read -r -p "Proceed? [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]] || { echo "Aborted."; exit 1; }
  fi
  sudo apt-get update
  sudo apt-get install -y "${MISSING[@]}"
fi

# ---------------------------------------------------------------- build
if [ "$BUILD" = 1 ]; then
  echo; echo ">>> colcon build"
  PKG_LIST=(limo_mocap_suite)
  if [ -n "$(find src -name package.xml -exec grep -l '<name>phasespace_msgs</name>' {} + 2>/dev/null)" ]; then
    PKG_LIST=(phasespace_msgs limo_mocap_suite)
  else
    echo "Note: phasespace_msgs is not in src/. Put the lab's phasespace_msgs package there"
    echo "      (or source its workspace) before running phasespace_bridge."
  fi
  colcon build --packages-select "${PKG_LIST[@]}"
  set +u; source install/setup.bash; set -u
fi

# ---------------------------------------------------------------- check again
echo; echo ">>> Dependency check (after)"
/usr/bin/python3 tools/check_deps.py && echo "Ready. In new terminals: source /opt/ros/${DISTRO}/setup.bash && source ${WS}/install/setup.bash"
