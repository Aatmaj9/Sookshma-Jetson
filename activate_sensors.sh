echo "🔄 Activating sensors..."

docker exec -d auv bash -ic 'source ~/.bashrc; dvl & ping360 & ping2 & sbg & modem & zed & uwb & velodyne &'

sleep 5

echo "🟢 All sensors activated!"

