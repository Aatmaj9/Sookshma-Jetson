# JP6.2 (L4T r36.4) + ZED SDK 5.0 + dev tools
FROM stereolabs/zed:5.0-tools-devel-l4t-r36.4

# ---------------------------------------------- ROS 2 Humble apt repo & base packages -----------------------------------------
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      python3-rosdep \
      python3-vcstool \
      python3-colcon-common-extensions \
      build-essential \
 && rosdep init || true \
 && rosdep update \
 && rm -rf /var/lib/apt/lists/*


RUN apt-get update
RUN apt-get -y install software-properties-common
RUN add-apt-repository universe
RUN apt-get update
RUN apt-get -y install python3-pip python3-serial python3-venv 
RUN apt-get -y install sudo xauth htop udev zip unzip curl
RUN apt-get update --fix-missing
RUN apt-get -y install libboost-all-dev libasio-dev 

# ------------------------------------------ System deps & ROS 2 packages in one layer ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential gcc gfortran git \
      libsm6 libxext6 libxrender1 libfontconfig1 libgl1 libboost-all-dev libasio-dev gdebi-core \
      ros-humble-diagnostic-updater \
      ros-humble-nmea-msgs \
      ros-humble-rtcm-msgs \
      ros-humble-rviz2 \
      ros-humble-rosbridge-server \
      ros-humble-image-transport-plugins \
      ros-humble-compressed-image-transport \
      ros-humble-compressed-depth-image-transport \
      ros-humble-theora-image-transport \
      ros-humble-point-cloud-transport-plugins \
      ros-humble-draco-point-cloud-transport \
      ros-humble-zlib-point-cloud-transport \
      ros-humble-zstd-point-cloud-transport \
      ros-humble-xacro \
      ros-humble-ffmpeg-image-transport \
      ros-humble-ffmpeg-encoder-decoder \
      ros-humble-zed-msgs \
      ros-humble-geographic-msgs \
      ros-humble-image-transport \
      ros-humble-point-cloud-transport \
      ros-humble-robot-localization \
      ros-humble-backward-ros \
    && ln -sf /usr/bin/python3 /usr/bin/python \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi bossa-cli && \
      rm -rf /var/lib/apt/lists/*



#-----------------------------------------------------------------------------------All basic installations is done---------------------------------------------------------------------------

# --- Create non-root user once (sudo + dialout) ---
ARG USERNAME=mavlab
ENV USERNAME=${USERNAME}
RUN useradd -m "${USERNAME}" \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && usermod --shell /bin/bash "${USERNAME}" \
    && usermod -aG sudo,dialout,video,plugdev "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

#---------------------------------------------------------------------------------Python Dependencies and  Arduino CLI---------------------------------------------------------------------

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# --- Python deps from requirements.txt -----
RUN python3 -m venv /home/${USERNAME}/.venv
ENV VIRTUAL_ENV=/home/${USERNAME}/.venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install your Python deps into the venv
COPY --chown=${USERNAME}:${USERNAME} requirements.txt /home/${USERNAME}/requirements.txt
RUN python3 -m pip install --upgrade pip \
 && python3 -m pip install --no-cache-dir -r /home/${USERNAME}/requirements.txt
RUN echo 'source $VIRTUAL_ENV/bin/activate' >> ~/.bashrc

# --- Arduino CLI Setup---
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
ENV PATH="/home/${USERNAME}/bin:${PATH}"
ENV ARDUINO_DATA="/home/${USERNAME}/.arduino15"
RUN arduino-cli config init \
 && arduino-cli config set directories.data "${ARDUINO_DATA}"
RUN arduino-cli core update-index && arduino-cli cache clean

# -----------------------------------------------------------------------------------Arduino Installations ----------------------------------------------------------------------------------

RUN arduino-cli core update-index && \
    arduino-cli cache clean && \
    arduino-cli core update-index --additional-urls https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json && \
    export ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS=https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json && \
    arduino-cli core install per1234:sam && \
    # Define a shell variable for the path
    SAM_ARM_PATH="/home/${USERNAME}/.arduino15/packages/per1234/hardware/sam/1.6.11-arm64" && \
    # Patch the platform.txt file
    curl -fsSL https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_arduinocore_sam.txt -o ${SAM_ARM_PATH}/platform.txt && \
    sed -i 's/-4.8.3-2014q1././g' ${SAM_ARM_PATH}/platform.txt && \
    # Install basic libraries
    arduino-cli lib install Servo && \
    git clone --depth=1 https://gitlab.com/timwilkinson/FlySkyIBus.git /home/${USERNAME}/Arduino/libraries/FlySkyIBus

COPY --chown=${USERNAME}:${USERNAME} micro_ros_arduino_arm.zip /tmp/micro_ros_arduino.zip


RUN printf "board_manager:\n  additional_urls: []\n\nlibrary:\n  enable_unsafe_install: true\n" > /home/${USERNAME}/.arduino15/arduino-cli.yaml && \
    # Install the zipped micro-ROS library
    arduino-cli --config-file /home/${USERNAME}/.arduino15/arduino-cli.yaml lib install --zip-path /tmp/micro_ros_arduino.zip && \
    rm /tmp/micro_ros_arduino.zip

# --------------------------------------------------------------------------------------Some Other Installations--------------------------------------------------------------------------------

# --- Node via NVM (user-local) + localtunnel ---
ENV NVM_DIR="/home/${USERNAME}/.nvm"    
ENV NODE_VERSION="20.17.0"
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash \
 && . "$NVM_DIR/nvm.sh" \
 && nvm install $NODE_VERSION \
 && nvm alias default $NODE_VERSION \
 && nvm use default \
 && npm install -g npm@latest localtunnel
ENV PATH="${NVM_DIR}/versions/node/v${NODE_VERSION}/bin:${PATH}"

#--------------------------------------------------------------------------------------------ROS-Convinience------------------------------------------------------------------------------------

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
 && echo 'if [ -f /workspaces/mavlab/ros2_ws/install/setup.bash ]; then source /workspaces/mavlab/ros2_ws/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc \
 && echo "alias sros2='source /opt/ros/humble/setup.bash && if [ -f /workspaces/mavlab/ros2_ws/install/setup.bash ]; then source /workspaces/mavlab/ros2_ws/install/setup.bash; fi'" >> /home/${USERNAME}/.bashrc \
 && echo "alias sbg='sros2 && ros2 launch sbg_driver sbg_device_launch.py'" >> /home/${USERNAME}/.bashrc \
 && echo "alias uwb='sros2 && ros2 run uwb_driver uwb'" >> /home/${USERNAME}/.bashrc \
 && echo "alias gnc='sros2 && ros2 run gnc gnc'" >> /home/${USERNAME}/.bashrc \
 && echo "alias mavsim='sros2 && ros2 run mav_simulator simulate'" >> /home/${USERNAME}/.bashrc \
 && echo "alias ardusimple='sros2 && ros2 launch ublox_gps ublox_gps_node-launch.py'" >> /home/${USERNAME}/.bashrc

# (Optional) Ports
EXPOSE 9000 9001 9002 9003
