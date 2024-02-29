#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: drone namespace, default is drone0"
    echo "      -e: estimator_type, choices: [gps_odometry, raw_odometry, mocap_pose, ground_truth]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
}

# Arg parser
while getopts "n:e:rt" opt; do
  case ${opt} in
    n )
      drone_namespace="${OPTARG}"
      ;;
    e )
      estimator_plugin="${OPTARG}"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

source utils/configure_dev_permissions.bash

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
estimator_plugin=${estimator_plugin:="gps_odometry"}
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}
drone_namespace=${drone_namespace:="dji_psdk"}

# Generate the list of drone namespaces
tmuxinator start -n ${drone_namespace} -p tmuxinator/aerostack.yml  drone_namespace=${drone_namespace} estimator_plugin=${estimator_plugin} &
wait

if [[ ${estimator_plugin} == "mocap_pose" ]]; then
  tmuxinator start -n mocap -p tmuxinator/mocap.yml &
  wait
fi

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -n rosbag -p tmuxinator/rosbag.yml drone_namespace=${drone_namespace} &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yml simulation=false drone_namespace=${drone_namespace} &
  wait
fi

# Attach to tmux session
tmux attach-session -t ${drone_namespace}:0