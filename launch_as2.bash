#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: select drone namespace to launch. Default is 'drone0'"
    echo "      -r: record rosbag. Default not launch"
    echo "      -g: launch using gnome-terminal instead of tmux. Default not set"
}

# Initialize variables with default values
drones_namespace="drone0"
rosbag="false"
use_gnome="false"

# Arg parser
while getopts "n:rg" opt; do
  case ${opt} in
    n )
      drones_namespace="${OPTARG}"
      ;;
    r )
      rosbag="true"
      ;;
    g )
      use_gnome="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# If no drone namespaces are provided, finish the execution
if [ -z "$drones_namespace" ]; then
  echo "No drone namespace provided. Set it using the -n option"
  exit 1
fi

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drones_namespace}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2
eval "tmuxinator ${tmuxinator_mode} -n drone -p tmuxinator/aerostack2.yaml \
  drone_namespace=${drones_namespace} \
  rosbag=${rosbag} \
  ${tmuxinator_end}"

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t drone
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi
