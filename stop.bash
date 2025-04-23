#!/bin/bash
tmux_session_list=()

# If not in a tmux session, add drone0 and ground_station to the list
if [[ -z "$TMUX" ]]; then
  tmux_session_list+=("drone" "ground_station")
fi

current_session=$(tmux display-message -p '#S')
stop_tmux_sessions="$(realpath "utils/stop_tmux_sessions.bash")"
${stop_tmux_sessions} "${tmux_session_list[@]}"
