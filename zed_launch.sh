ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i &
sleep 3

ros2 run foxglove_bridge foxglove_bridge --port 8765