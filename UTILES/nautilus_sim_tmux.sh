#!/usr/bin/env bash

source ~/.bashrc
shopt -s expand_aliases

build || exit 1

SESSION="AUV"
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session already exists"
    tmux attach-session -t "$SESSION"
else
    echo "Creating session"
    tmux new-session -d -s "$SESSION" -n "bringup"
fi

# Split into 3 panes
tmux split-window -h -t "$SESSION:bringup"
tmux split-window -v -t "$SESSION:bringup.1"

# Run commands in each pane
tmux send-keys -t "$SESSION:bringup.0" "ros2 run rqt_image_view rqt_image_view & ros2 launch nautilus_bringup orca_comp.launch.py" C-m
tmux send-keys -t "$SESSION:bringup.1" "ros2 run nautilus_sensors yolo_pipeline --sim --bbox" C-m
tmux send-keys -t "$SESSION:bringup.2" "ros2 launch nautilus_robot_localization ekf.launch.py fake_dvl:='true'" C-m

# Optional: make panes evenly sized
tmux select-layout -t "$SESSION:bringup" tiled

# Create second window
tmux new-window -t "$SESSION" -n "mission"

# Split into 4 panes
tmux split-window -h -t "$SESSION:mission"
tmux split-window -v -t "$SESSION:mission.0"
tmux split-window -v -t "$SESSION:mission.1"

# Run commands in each pane
tmux send-keys -t "$SESSION:mission.0" "ros2 launch nautilus_controls vision_controller_sim.launch.py" C-m
tmux send-keys -t "$SESSION:mission.1" "ros2 run nautilus_controls cube_interface --sitl" C-m
tmux send-keys -t "$SESSION:mission.2" "ros2 run nautilus_mission master_node"

# Optional: make panes evenly sized
tmux select-layout -t "$SESSION:mission" tiled

# Attach to session
tmux attach-session -t "$SESSION"
