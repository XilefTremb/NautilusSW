#!/usr/bin/env bash
# Run in a subshell so sourcing this script never exits the caller.
(
shopt -s expand_aliases
source ~/.bashrc
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR/../nautilus_ws"
t() { tmux -L nautilus-sim "$@"; }
if t has-session -t AUV 2>/dev/null; then
    echo "Simulation already running; attaching."
    t attach-session -t AUV
    exit
fi
systemctl --user show-environment >/dev/null || { echo "A systemd user session is required."; exit 1; }
if systemctl --user is-active --quiet nautilus-sim.scope; then
    echo "Previous simulation still running. Run nautilus_sim_stop.sh first."
    exit 1
fi
build
# A dedicated tmux server ensures all panes and their descendants inherit the scope.
systemd-run --user --scope --unit=nautilus-sim --property=TimeoutStopSec=10s tmux -L nautilus-sim new-session -d -s AUV -n bringup
trap 'bash "$DIR/nautilus_sim_stop.sh"' ERR
t split-window -h -t AUV:bringup.0
t split-window -v -t AUV:bringup.1
t send-keys -t AUV:bringup.0 "ros2 run rqt_image_view rqt_image_view & ros2 launch nautilus_bringup orca_comp.launch.py" C-m
t send-keys -t AUV:bringup.1 "ros2 run nautilus_sensors yolo_pipeline --sim --bbox" C-m
t send-keys -t AUV:bringup.2 "ros2 launch nautilus_robot_localization ekf.launch.py fake_dvl:='true'" C-m
t select-layout -t AUV:bringup tiled
t new-window -t AUV -n mission
right_pane=$(t split-window -h -t AUV:mission.0 -P -F '#{pane_id}')
t split-window -v -t AUV:mission.0
t split-window -v -t "$right_pane"
t select-layout -t AUV:mission tiled
t send-keys -t AUV:mission.0 "ros2 launch nautilus_controls vision_controller_sim.launch.py" C-m
t send-keys -t AUV:mission.1 "ros2 run nautilus_controls cube_interface --sitl" C-m
# Keep these two commands ready for manual execution, as before.
t send-keys -t AUV:mission.2 "ros2 run nautilus_mission master_node -- --sim"
t send-keys -t AUV:mission.3 "ros2 run nautilus_sensors yolo_detections_viewer --ros-args -p detection_topic:=/yolo/detections_forward"
# Launch the stop script in a separate systemd service, outside the simulation scope.
printf -v stop_cmd '%q' "$DIR/nautilus_sim_stop.sh"
t bind-key K confirm-before -p "Stop Nautilus simulation? (y/n)" "run-shell -b 'systemd-run --user --collect /bin/bash $stop_cmd'"
trap - ERR
t attach-session -t AUV
)
