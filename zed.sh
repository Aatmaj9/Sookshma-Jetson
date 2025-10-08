ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i &
sleep 3

# Start the Foxglove WebSocket bridge on port 8765 (default)
ros2 run foxglove_bridge foxglove_bridge --port 8765