#!/bin/bash

# Get file name from argument. Default use config.yaml
if [ "$#" -eq 1 ]; then
  CONFIG_FILE="$1"
else
  CONFIG_FILE="config/rosbag_topics.yaml"
fi

# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configuration file path (absolute path)
CONFIG_FILE="$SCRIPT_DIR/$CONFIG_FILE"

# Check if the config file exists
if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "Error: Configuration file not found in $CONFIG_FILE"
  exit 1
fi

# Create directory for rosbag storage
mkdir -p rosbags
cd rosbags || exit

# Generate a unique name with fixed prefix and two-digit number
prefix="flight_"
for i in $(seq -w 0 99); do
  filename="${prefix}${i}"
  if [[ ! -e "$filename" ]]; then
    ROSBAG_NAME="$filename"
    break
  fi
done

# If no available name was found
if [[ -z "$ROSBAG_NAME" ]]; then
  echo "Error: Could not find an available name for the rosbag (flight_00 to flight_99 are taken)"
  exit 1
fi

# Construct the rosbag record command
rosbag_cmd="ros2 bag record"

# Read topics from the config file, ignoring comments (#) and disabled topics (!)
while IFS= read -r topic; do
  [[ "$topic" =~ ^#.*$ || "$topic" =~ ^!.*$ || -z "$topic" ]] && continue  # Skip comments, disabled topics, and empty lines
  rosbag_cmd+=" ${topic}"
done < "$CONFIG_FILE"

# Check if any topics were added
if [[ "$rosbag_cmd" == "ros2 bag record" ]]; then
  echo "Warning: No valid topics found in $CONFIG_FILE"
  exit 1
fi

# Add output name to the command
rosbag_cmd+=" -o ${ROSBAG_NAME}"

echo "Rosbag name: ${ROSBAG_NAME}"
echo "Starting rosbag recording..."

# Execute the rosbag record command
eval "$rosbag_cmd"
