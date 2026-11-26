#!/usr/bin/env bash
(
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$DIR/nautilus_sim_stop.sh" && bash "$DIR/nautilus_sim_tmux.sh"
)
