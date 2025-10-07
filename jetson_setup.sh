#auto-connect to wifi and set hostname
nmcli device wifi list
nmcli device wifi connect mavlab password mavlab24
nmcli connection modify mavlab connection.autoconnect yes
sudo hostnamectl set-hostname masv01

#install git
sudo apt-get update
sudo apt-get install git -y

#add user to docker group
sudo usermod -aG docker $USER


