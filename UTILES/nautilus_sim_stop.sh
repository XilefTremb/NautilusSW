#!/usr/bin/env bash
(
set -e
unit=nautilus-sim.scope
systemctl --user show-environment >/dev/null || exit 1
if tmux -L nautilus-sim has-session -t AUV 2>/dev/null; then
    echo "Requesting graceful ROS shutdown..."
    while IFS= read -r pane; do
        tmux -L nautilus-sim send-keys -t "$pane" C-c || true
    done < <(tmux -L nautilus-sim list-panes -s -t AUV -F '#{pane_id}')
    sleep 10
fi
if systemctl --user is-active --quiet "$unit"; then
    echo "Terminating remaining simulation processes..."
    # Scope stop sends SIGTERM, then SIGKILL after TimeoutStopSec.
    systemctl --user stop "$unit"
fi
if tmux -L nautilus-sim has-session -t AUV 2>/dev/null; then
    tmux -L nautilus-sim kill-session -t AUV
fi
if systemctl --user is-active --quiet "$unit"; then
    echo "Simulation scope still active; inspect systemctl --user status $unit"
    exit 1
fi
echo "Nautilus simulation stopped."
)
