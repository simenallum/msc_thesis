#!/bin/bash

tmux send-keys "cd ~/" Enter
. source_noetic.sh

tmux split-window -v
. source_noetic.sh

tmux split-window -h
. source_noetic.sh

tmux select-pane -t 0

tmux split-window -h
. source_noetic.sh