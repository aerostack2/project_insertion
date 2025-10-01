ros2 bag video --progress -t /M350/sensor_measurements/main_camera/image_raw  -o ./flight_23.mp4 flight_23

ros2 bag extract --progress -t /M350/sensor_measurements/main_camera/image_raw -i flight_23
