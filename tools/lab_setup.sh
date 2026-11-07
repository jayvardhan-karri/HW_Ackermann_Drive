#!/usr/bin/env bash
# =============================================================================
# lab_setup.sh - one-step terminal setup on the LAB machine (bash only)
#
# Purpose
#   Prepare a lab terminal for the LIMO / PhaseSpace work: source ROS 2 Foxy and
#   the workspaces, choose ROS_DOMAIN_ID, and optionally launch PhaseSpace.
#
# Usage (recommended: SOURCE it so the exports stay in your terminal)
#   source tools/lab_setup.sh
#   (executing it, ./tools/lab_setup.sh, also works, but the exports are lost
#    when the script ends; PhaseSpace still launches with the right environment)
#
# What it does
#   [1/4] sources /opt/ros/foxy/setup.bash (FAIL + stop if missing)
#   [2/4] sources ~/ros2_ws/install/setup.bash (phasespace_bringup/_msgs; WARN
#         if missing) and <repo>/install/setup.bash if this repo is built
#   [3/4] asks for ROS_DOMAIN_ID (0-101) and exports it
#   [4/4] asks "Launch PhaseSpace now? [Y/n]" and runs
#         ros2 launch phasespace_bringup phasespace_mocap.launch.py
#         or prints a summary with the next commands
#
# Assumptions: Ubuntu 20.04, ROS 2 Foxy, lab workspace in ~/ros2_ws, bash.
#
# Test overrides (testing only; leave unset in the lab)
#   LAB_SETUP_ROS_SETUP  default /opt/ros/foxy/setup.bash
#   LAB_SETUP_WS_SETUP   default ~/ros2_ws/install/setup.bash
# =============================================================================

# Detect sourced vs executed (must be evaluated at top level)
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    _LAB_SETUP_SOURCED=1
else
    _LAB_SETUP_SOURCED=0
fi
_LAB_SETUP_FILE="${BASH_SOURCE[0]}"

_lab_setup_main() {
    local sourced="$_LAB_SETUP_SOURCED"
    local root ros_setup ws_setup repo_setup
    root="$(cd "$(dirname "$_LAB_SETUP_FILE")/.." && pwd)"
    ros_setup="${LAB_SETUP_ROS_SETUP:-/opt/ros/foxy/setup.bash}"
    ws_setup="${LAB_SETUP_WS_SETUP:-$HOME/ros2_ws/install/setup.bash}"
    repo_setup="$root/install/setup.bash"

    # --- colours (fall back to plain text) ---
    local G="" Y="" R="" B="" N=""
    if [[ -t 1 ]] && command -v tput >/dev/null 2>&1 \
        && [[ "$(tput colors 2>/dev/null || echo 0)" -ge 8 ]]; then
        G="$(tput setaf 2)"; Y="$(tput setaf 3)"; R="$(tput setaf 1)"
        B="$(tput bold)"; N="$(tput sgr0)"
    fi
    local EQ="================================================================"
    local DASH="----------------------------------------------------------------"
    ok()   { printf '  [%sOK%s]   %s\n'   "$G" "$N" "$*"; }
    warn() { printf '  [%sWARN%s] %s\n'   "$Y" "$N" "$*"; }
    fail() { printf '  [%sFAIL%s] %s\n'   "$R" "$N" "$*"; }
    step() { printf '\n%s\n%s%s%s\n' "$DASH" "$B" "$*" "$N"; }

    # --- Ctrl-C handling: never kill the user's shell when sourced ---
    local old_int
    old_int="$(trap -p INT)"
    _lab_restore_trap() { if [[ -n "$old_int" ]]; then eval "$old_int"; else trap - INT; fi; }
    trap 'echo; echo "  Interrupted."' INT

    # ask <prompt> <varname>: returns 1 on EOF/interrupt
    ask() {
        local __ans
        if ! IFS= read -r -p "$1" __ans; then
            echo
            return 1
        fi
        printf -v "$2" '%s' "$__ans"
    }

    printf '%s\n%s%s%s\n%s\n' "$EQ" "$B" "  LAB SETUP  (LIMO / PhaseSpace, ROS 2 Foxy)" "$N" "$EQ"

    # ---------------- [1/4] ROS 2 Foxy ----------------
    step "[1/4] ROS 2 Foxy"
    if [[ ! -f "$ros_setup" ]]; then
        fail "ROS 2 setup not found: $ros_setup"
        echo "        Install ROS 2 Foxy or fix the path, then run again."
        _lab_restore_trap; unset -f ok warn fail step ask _lab_restore_trap
        return 1
    fi
    # shellcheck disable=SC1090
    source "$ros_setup"
    ok "sourced $ros_setup (ROS_DISTRO=${ROS_DISTRO:-unknown})"

    # ---------------- [2/4] Workspaces ----------------
    step "[2/4] Workspaces"
    local sourced_list="$ros_setup"
    if [[ -f "$ws_setup" ]]; then
        # shellcheck disable=SC1090
        source "$ws_setup"
        ok "sourced lab workspace $ws_setup"
        sourced_list="$sourced_list, $ws_setup"
    else
        warn "lab workspace not found: $ws_setup"
        echo "        PhaseSpace launch and the bridge need it (phasespace_bringup, phasespace_msgs)."
    fi
    if [[ -f "$repo_setup" ]]; then
        # shellcheck disable=SC1090
        source "$repo_setup"
        ok "sourced repo workspace $repo_setup"
        sourced_list="$sourced_list, $repo_setup"
    else
        warn "repo workspace not built: $repo_setup"
        echo "        cd $root && colcon build --packages-select limo_mocap_suite"
    fi

    # ---------------- [3/4] ROS_DOMAIN_ID ----------------
    step "[3/4] ROS_DOMAIN_ID"
    local def="${ROS_DOMAIN_ID:-0}" dom=""
    while true; do
        if ! ask "ROS_DOMAIN_ID (0-101) [$def]: " dom; then
            fail "no input, aborting"
            _lab_restore_trap; unset -f ok warn fail step ask _lab_restore_trap
            return 1
        fi
        [[ -z "$dom" ]] && dom="$def"
        if [[ "$dom" =~ ^[0-9]+$ ]] && (( 10#$dom >= 0 && 10#$dom <= 101 )); then
            dom=$((10#$dom))
            break
        fi
        warn "invalid value '$dom' (need an integer 0..101)"
    done
    export ROS_DOMAIN_ID="$dom"
    ok "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

    # ---------------- [4/4] PhaseSpace ----------------
    step "[4/4] PhaseSpace"
    local yn="" rc=0
    if ! ask "Launch PhaseSpace now? [Y/n]: " yn; then
        fail "no input, aborting"
        _lab_restore_trap; unset -f ok warn fail step ask _lab_restore_trap
        return 1
    fi
    if [[ -z "$yn" || "$yn" =~ ^[Yy] ]]; then
        if ! ros2 pkg prefix phasespace_bringup >/dev/null 2>&1; then
            fail "package phasespace_bringup not found"
            echo "        Build and source the lab workspace: cd ~/ros2_ws && colcon build && source install/setup.bash"
            rc=1
        else
            echo "  Running: ros2 launch phasespace_bringup phasespace_mocap.launch.py"
            echo "$DASH"
            _lab_restore_trap   # let ros2 launch handle Ctrl-C itself
            ros2 launch phasespace_bringup phasespace_mocap.launch.py
            rc=$?
            echo "$DASH"
            echo "  PhaseSpace launch ended (exit code $rc)."
        fi
    else
        printf '\n%s\n%s%s%s\n%s\n' "$EQ" "$B" "  Setup done" "$N" "$EQ"
        echo "  ROS distro      : ${ROS_DISTRO:-unknown}"
        echo "  Sourced         : $sourced_list"
        echo "  ROS_DOMAIN_ID   : $ROS_DOMAIN_ID"
        echo
        echo "  Next commands in this shell:"
        echo "    ros2 launch phasespace_bringup phasespace_mocap.launch.py"
        echo "    ros2 launch limo_mocap_suite hw_check.launch.py"
        echo "    ros2 launch limo_mocap_suite record_lap.launch.py"
    fi

    if (( ! sourced )); then
        echo
        echo "  NOTE: this script was executed, not sourced. The exports (ROS_DOMAIN_ID,"
        echo "        sourced workspaces) do NOT stay in your terminal. Use instead:"
        echo "          source tools/lab_setup.sh"
    fi
    echo "$EQ"
    _lab_restore_trap
    unset -f ok warn fail step ask _lab_restore_trap
    return "$rc"
}

_lab_setup_main
_LAB_SETUP_RC=$?
_lab_setup_sourced_flag="$_LAB_SETUP_SOURCED"
unset -f _lab_setup_main
unset _LAB_SETUP_SOURCED _LAB_SETUP_FILE
if (( _lab_setup_sourced_flag )); then
    # expand the code first, then unset the helpers and return (never exit)
    eval "unset _LAB_SETUP_RC _lab_setup_sourced_flag; return $_LAB_SETUP_RC"
else
    exit "$_LAB_SETUP_RC"
fi
