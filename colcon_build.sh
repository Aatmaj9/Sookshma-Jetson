cd ./ros2_ws
sudo rm -rf build log install
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
sros2
cd -