#!/bin/bash

DRIVER_TITLE="team_driver"

# ------------------------------------------------------------
# Require tmux
# ------------------------------------------------------------

if [ -z "$TMUX" ]; then
    echo -e "\033[31m"
    echo "ERROR: This launch must be run inside tmux."
    echo
    echo "Start tmux first:"
    echo "    tmux"
    echo
    echo "Then run:"
    echo "    ros2 launch team_driver driver.launch.py"
    echo -e "\033[0m"
    exit 1
fi

# ------------------------------------------------------------
# Check if the driver pane already exists
# ------------------------------------------------------------

DRIVER_PANE=$(tmux list-panes -F '#{pane_id} #{pane_title}' \
    | awk -v title="$DRIVER_TITLE" '$2 == title {print $1; exit}')

if [ -n "$DRIVER_PANE" ]; then
    # Reuse existing driver pane
    tmux send-keys -t "$DRIVER_PANE" C-c
    tmux send-keys -t "$DRIVER_PANE" "$*" C-m
    exit 0
fi


# ------------------------------------------------------------
# Create driver pane
# ------------------------------------------------------------

DRIVER_PANE=$(tmux split-window \
    -v \
    -P \
    -F '#{pane_id}' \
    -d)

tmux select-pane \
    -t "$DRIVER_PANE" \
    -T "$DRIVER_TITLE"


# ------------------------------------------------------------
# Run driver
# ------------------------------------------------------------

tmux send-keys \
    -t "$DRIVER_PANE" \
    "$*" \
    C-m
