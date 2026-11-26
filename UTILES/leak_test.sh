leak_test() (
    # Keep process-group IDs in a temporary directory
    local runtime_dir
    runtime_dir=$(mktemp -d) || exit 1

    cleanup() {
        trap - EXIT INT TERM HUP

        echo "Stopping ROS 2 processes..."

        local pgid

        for file in "$runtime_dir"/*.pgid; do
            [[ -f "$file" ]] || continue
            read -r pgid < "$file"

            if [[ "$pgid" =~ ^[0-9]+$ ]]; then
                kill -INT -- "-$pgid" 2>/dev/null
            fi
        done

        sleep 2

        for file in "$runtime_dir"/*.pgid; do
            [[ -f "$file" ]] || continue
            read -r pgid < "$file"

            if [[ "$pgid" =~ ^[0-9]+$ ]]; then
                kill -KILL -- "-$pgid" 2>/dev/null
            fi
        done

        wait 2>/dev/null
        rm -rf -- "$runtime_dir"

        echo "Cleanup complete."
    }

    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP

    # Start Cube interface in a new session
    setsid bash -c '
        echo "$$" > "$1"
        exec ros2 run nautilus_controls cube_interface --auv
    ' bash "$runtime_dir/cube.pgid" &

    # Start MAVProxy router in a new session
    setsid bash -c '
        echo "$$" > "$1"
        exec ros2 launch nautilus_bringup mavproxy_router.launch.py
    ' bash "$runtime_dir/router.pgid" &

    # Wait for PID files to be created
    for file in "$runtime_dir/cube.pgid" "$runtime_dir/router.pgid"; do
        for i in {1..20}; do
            [[ -s "$file" ]] && break
            sleep 0.1
        done

        if [[ ! -s "$file" ]]; then
            echo "ERROR: Failed to identify process group for $file"
            exit 1
        fi
    done

    # Launch HMI
    python3 ~/NautilusSW/nautilus_ws/src/nautilus_sensors/HMI/main.py -l
)
    