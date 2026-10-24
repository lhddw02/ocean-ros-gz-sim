# syntax=docker/dockerfile:1
ARG ROS_IMAGE=ros:humble-ros-base-jammy
FROM ${ROS_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG ASV_REF=ca8629df4e191235753dfae92ef725d30b923364
ARG BUILD_JOBS=2
ENV GZ_VERSION=harmonic \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=30 \
    GZ_PARTITION=ocean-sim \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1

# Humble normally selects Fortress. Install the explicit OSRF Harmonic bridge.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg git \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic ros-humble-ros-gzharmonic \
      ros-humble-rmw-cyclonedds-cpp ros-humble-tf2-msgs \
      ros-humble-rosgraph-msgs ros-humble-launch-ros ros-humble-rosbag2 \
      build-essential cmake ninja-build pkg-config \
      python3-colcon-common-extensions python3-yaml \
      libcgal-dev libfftw3-dev libeigen3-dev \
      libboost-iostreams-dev libboost-filesystem-dev \
      mesa-utils libgl1-mesa-dri libglx-mesa0 libegl-mesa0 \
    && test -f /usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/asv_ws
COPY patches /opt/ocean/patches
RUN mkdir -p src/asv_wave_sim \
    && git -C src/asv_wave_sim init \
    && git -C src/asv_wave_sim remote add origin https://github.com/srmainwaring/asv_wave_sim.git \
    && git -C src/asv_wave_sim fetch --depth 1 origin "${ASV_REF}" \
    && git -C src/asv_wave_sim checkout --detach FETCH_HEAD \
    && test "$(git -C src/asv_wave_sim rev-parse HEAD)" = "${ASV_REF}" \
    && git -C src/asv_wave_sim apply --check /opt/ocean/patches/asv-wave-visual-dedup.patch \
    && git -C src/asv_wave_sim apply /opt/ocean/patches/asv-wave-visual-dedup.patch \
    && git -C src/asv_wave_sim apply --check /opt/ocean/patches/asv-wave-reset.patch \
    && git -C src/asv_wave_sim apply /opt/ocean/patches/asv-wave-reset.patch \
    && source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}" MAKEFLAGS="-j${BUILD_JOBS}" \
      colcon build --merge-install --executor sequential \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
                   -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_LIBDIR=lib \
    && test -f install/lib/libgz-waves1-waves-model-system.so \
    && test -f install/lib/libgz-waves1-waves-visual-system.so \
    && test -f install/lib/libgz-waves1-hydrodynamics-system.so \
    && test -f install/lib/libgz-waves1-rendering-ogre2.so \
    && rm -rf build log

# Keep camera/vision dependencies after the expensive, pinned ASV build layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-opencv python3-numpy ros-humble-sensor-msgs ros-humble-std-msgs \
    && python3 -c "import cv2; assert hasattr(cv2, 'aruco'); assert hasattr(cv2.aruco, 'DICT_APRILTAG_36h11')" \
    && rm -rf /var/lib/apt/lists/*

COPY ros /opt/ocean_ws/src
WORKDIR /opt/ocean_ws
RUN source /opt/ros/humble/setup.bash \
    && source /opt/asv_ws/install/setup.bash \
    && PYTHONPATH=/opt/ocean_ws/src/ocean_sim python3 -c \
       "from ocean_sim.scene_builder import generate_tag; generate_tag('/opt/ocean_ws/src/ocean_sim/models/floating_station/materials/textures/apriltag.png', 'tag36h11', 0, 0.4, 0.1)" \
    && colcon build --merge-install --executor sequential \
         --packages-select ocean_station_systems ocean_landing_gui ocean_sim \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && test -f install/lib/libocean-station-mooring-system.so \
    && test -f install/lib/libocean-replay-contact-system.so \
    && test -f install/lib/libLandingReplay.so \
    && python3 /opt/ocean_ws/src/ocean_sim/ocean_sim/scene_builder.py \
         --share /opt/ocean_ws/install/share/ocean_sim --output /opt/ocean/default-landing \
    && rm -rf build log

COPY container /opt/ocean/container
COPY third_party /opt/ocean/third_party
COPY scripts/set-waves.sh /usr/local/bin/set-waves
RUN sed -i 's/\r$//' /opt/ocean/container/*.sh /usr/local/bin/set-waves \
    && chmod +x /opt/ocean/container/*.sh /usr/local/bin/set-waves \
    && printf '%s\n' "ROS=humble" "GAZEBO=harmonic" "ASV_REF=${ASV_REF}" \
       "ASV_PATCH=asv-wave-visual-dedup.patch,asv-wave-reset.patch" \
       > /opt/ocean/build-info.txt \
    && dpkg-query -W >> /opt/ocean/build-info.txt

WORKDIR /opt/ocean
ENTRYPOINT ["/opt/ocean/container/entrypoint.sh"]
CMD ["ros2", "launch", "ocean_sim", "ocean.launch.py"]
