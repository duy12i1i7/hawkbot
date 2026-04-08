#!/usr/bin/env bash
###############################################################################
# HawkBot ROS2 — All-in-One Installation Script
#
# This script installs everything needed to build and run the HawkBot project:
#   1. ROS2 Humble (if not already installed)
#   2. System apt dependencies
#   3. Python pip dependencies
#   4. YDLidar SDK (system-wide)
#   5. Source dependencies (ydlidar_ros2_driver, imu_tools)
#   6. LiDAR udev rules + user permissions
#   7. colcon workspace build
#
# Usage:
#   chmod +x install.sh
#   ./install.sh
#
# Options:
#   --skip-ros2       Skip ROS2 Humble installation (if already installed)
#   --skip-build      Skip colcon build step
#   --jobs N          Number of parallel make jobs (default: auto, use 1 for low RAM)
#
###############################################################################
set -euo pipefail

# ── Color helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()  { echo -e "${BLUE}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()   { echo -e "${RED}[ERROR]${NC} $*"; }

# ── Parse arguments ──────────────────────────────────────────────────────────
SKIP_ROS2=false
SKIP_BUILD=false
MAKE_JOBS=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-ros2)  SKIP_ROS2=true; shift ;;
        --skip-build) SKIP_BUILD=true; shift ;;
        --jobs)       MAKE_JOBS="$2"; shift 2 ;;
        *)            err "Unknown option: $1"; exit 1 ;;
    esac
done

# ── Detect workspace root ───────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# If script is inside src/, workspace root is one level up
if [[ "$(basename "$SCRIPT_DIR")" == "src" ]]; then
    WS_ROOT="$(dirname "$SCRIPT_DIR")"
    SRC_DIR="$SCRIPT_DIR"
elif [[ -d "$SCRIPT_DIR/src" ]]; then
    WS_ROOT="$SCRIPT_DIR"
    SRC_DIR="$SCRIPT_DIR/src"
else
    WS_ROOT="$HOME/ROS2_WS"
    SRC_DIR="$WS_ROOT/src"
fi

info "Workspace root: $WS_ROOT"
info "Source dir:      $SRC_DIR"

# ── Helper: check command exists ─────────────────────────────────────────────
has_cmd() { command -v "$1" &>/dev/null; }

# Source ROS setup with nounset disabled to avoid unbound var issues
# from upstream setup scripts when this installer runs with `set -u`.
source_ros_setup() {
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
}

# ── 1. Install ROS2 Humble ──────────────────────────────────────────────────
install_ros2() {
    if [[ "$SKIP_ROS2" == true ]]; then
        warn "Skipping ROS2 installation (--skip-ros2)"
        return
    fi

    if [[ -f /opt/ros/humble/setup.bash ]]; then
        ok "ROS2 Humble already installed"
        return
    fi

    info "Installing ROS2 Humble..."

    sudo apt update && sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe -y

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-humble-desktop

    ok "ROS2 Humble installed"
}

# ── 2. Install system dependencies ──────────────────────────────────────────
install_apt_deps() {
    info "Installing apt dependencies..."

    # Source ROS2 for rosdep
    source_ros_setup

    sudo apt update
    sudo apt install -y \
        ros-humble-cartographer \
        ros-humble-cartographer-ros \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-xacro \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        cmake \
        build-essential \
        git

    # Initialize rosdep
    sudo rosdep init 2>/dev/null || true
    rosdep update --rosdistro=humble || true

    ok "apt dependencies installed"
}

# ── 3. Install Python dependencies ──────────────────────────────────────────
install_pip_deps() {
    info "Installing Python pip dependencies..."

    pip3 install --user opencv-python mediapipe numpy 2>/dev/null || \
        pip3 install opencv-python mediapipe numpy

    ok "Python dependencies installed"
}

# ── 4. Install YDLidar SDK ──────────────────────────────────────────────────
install_ydlidar_sdk() {
    # Check if already installed
    if [[ -f /usr/local/lib/libydlidar_sdk.a ]] || [[ -f /usr/local/lib/libydlidar_sdk.so ]]; then
        ok "YDLidar SDK already installed"
        return
    fi

    info "Installing YDLidar SDK..."

    local tmp_dir
    tmp_dir="$(mktemp -d)"

    git clone https://github.com/YDLIDAR/YDLidar-SDK.git "$tmp_dir/YDLidar-SDK"
    cd "$tmp_dir/YDLidar-SDK"
    mkdir -p build && cd build
    cmake ..
    make -j"$(nproc)"
    sudo make install

    # Cleanup
    rm -rf "$tmp_dir"
    cd "$SRC_DIR"

    ok "YDLidar SDK installed to /usr/local"
}

# ── 5. Clone source dependencies ────────────────────────────────────────────
clone_source_deps() {
    info "Checking source dependencies..."

    mkdir -p "$SRC_DIR"
    cd "$SRC_DIR"

    # ydlidar_ros2_driver
    if [[ ! -d "$SRC_DIR/ydlidar_ros2_driver" ]]; then
        info "Cloning ydlidar_ros2_driver (humble branch)..."
        git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
        ok "ydlidar_ros2_driver cloned"
    else
        ok "ydlidar_ros2_driver already exists"
    fi

    # imu_tools (provides imu_complementary_filter, imu_filter_madgwick)
    if [[ ! -d "$SRC_DIR/imu_tools" ]]; then
        info "Cloning imu_tools (humble branch)..."
        git clone -b humble https://github.com/CCNYRoboticsLab/imu_tools.git
        ok "imu_tools cloned"
    else
        ok "imu_tools already exists"
    fi
}

# ── 6. Download model files ──────────────────────────────────────────────────
download_models() {
    info "Checking model files..."

    local dat_file="$SRC_DIR/hawkbot_mediapipe/hawkbot_mediapipe/file/shape_predictor_68_face_landmarks.dat"
    if [[ -f "$dat_file" ]]; then
        ok "Face detection model already exists"
    else
        info "Downloading face detection model (shape_predictor_68_face_landmarks.dat ~96 MB)..."
        mkdir -p "$(dirname "$dat_file")"
        wget -q --show-progress -O "${dat_file}.bz2" \
            http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
        bzip2 -d "${dat_file}.bz2"
        ok "Face detection model downloaded"
    fi
}

# ── 7. Set up LiDAR permissions ─────────────────────────────────────────────
setup_lidar_permissions() {
    info "Setting up LiDAR USB permissions..."

    # Add user to dialout group
    if groups "$USER" | grep -q dialout; then
        ok "User already in dialout group"
    else
        sudo usermod -a -G dialout "$USER"
        warn "Added $USER to dialout group — log out and back in for it to take effect"
    fi

    # udev rule for YDLidar
    local UDEV_RULE='KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"'
    local UDEV_FILE="/etc/udev/rules.d/ydlidar.rules"

    if [[ -f "$UDEV_FILE" ]]; then
        ok "udev rule already exists"
    else
        echo "$UDEV_RULE" | sudo tee "$UDEV_FILE" > /dev/null
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        ok "udev rule created at $UDEV_FILE"
    fi
}

# ── 8. Build the workspace ──────────────────────────────────────────────────
build_workspace() {
    if [[ "$SKIP_BUILD" == true ]]; then
        warn "Skipping build (--skip-build)"
        return
    fi

    info "Building workspace at $WS_ROOT ..."

    cd "$WS_ROOT"
    source_ros_setup

    # Determine make jobs
    local make_flags=""
    local parallel_flags=""

    if [[ -n "$MAKE_JOBS" ]]; then
        make_flags="MAKEFLAGS=-j${MAKE_JOBS}"
        parallel_flags="--parallel-workers ${MAKE_JOBS}"
        info "Using --jobs $MAKE_JOBS (limited parallelism)"
    else
        # Auto-detect: if RAM < 6GB, limit to 1 job
        local mem_kb
        mem_kb=$(grep MemTotal /proc/meminfo | awk '{print $2}')
        if (( mem_kb < 6000000 )); then
            make_flags="MAKEFLAGS=-j1"
            parallel_flags="--parallel-workers 2"
            warn "Low RAM detected ($(( mem_kb / 1024 )) MB) — limiting build parallelism"
        fi
    fi

    # Run build
    if [[ -n "$make_flags" ]]; then
        env "$make_flags" colcon build --symlink-install $parallel_flags
    else
        colcon build --symlink-install
    fi

    ok "Workspace built successfully"
}

# ── 9. Set up shell environment ──────────────────────────────────────────────
setup_shell() {
    local bashrc="$HOME/.bashrc"
    local ros_source="source /opt/ros/humble/setup.bash"
    local ws_source="source $WS_ROOT/install/setup.bash"

    if ! grep -qF "$ros_source" "$bashrc" 2>/dev/null; then
        echo "$ros_source" >> "$bashrc"
        info "Added ROS2 source to ~/.bashrc"
    fi

    if ! grep -qF "$ws_source" "$bashrc" 2>/dev/null; then
        echo "$ws_source" >> "$bashrc"
        info "Added workspace source to ~/.bashrc"
    fi

    ok "Shell environment configured"
}

# ── Main ─────────────────────────────────────────────────────────────────────
main() {
    echo ""
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║         HawkBot ROS2 — All-in-One Installer             ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo ""

    install_ros2
    install_apt_deps
    install_pip_deps
    install_ydlidar_sdk
    clone_source_deps
    download_models
    setup_lidar_permissions
    build_workspace
    setup_shell

    echo ""
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║                  Installation Complete!                  ║"
    echo "╠═══════════════════════════════════════════════════════════╣"
    echo "║                                                         ║"
    echo "║  Next steps:                                            ║"
    echo "║                                                         ║"
    echo "║  1. Open a new terminal (or run: source ~/.bashrc)      ║"
    echo "║                                                         ║"
    echo "║  2. Launch the robot:                                   ║"
    echo "║     ros2 launch hawkbot bringup_launch.py ip:=<IP>      ║"
    echo "║                                                         ║"
    echo "║  3. Teleop (in another terminal):                       ║"
    echo "║     ros2 run hawkbot teleop_keyboard                    ║"
    echo "║                                                         ║"
    echo "║  4. SLAM:                                               ║"
    echo "║     ros2 launch hawkbot_cartographer cartographer.launch.py ║"
    echo "║                                                         ║"
    echo "║  5. Navigation:                                         ║"
    echo "║     ros2 launch hawkbot_navigation2 navigation2.launch.py   ║"
    echo "║                                                         ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo ""

    if ! groups "$USER" | grep -q dialout; then
        warn "Remember to log out and back in for LiDAR USB permissions!"
    fi
}

main "$@"
