ARG NAVIGATION_BASE_IMAGE=ros2_ws-ros2_nav2:latest
FROM ${NAVIGATION_BASE_IMAGE}

RUN apt-get update && apt-get install -y python3-pytest && rm -rf /var/lib/apt/lists/*

WORKDIR /lab
RUN . /opt/ros/humble/setup.sh && \
    CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --event-handlers console_direct+ \
      --base-paths /opt/fields2cover_src /opt/opennav_coverage_src \
                   /ros2_ws/src/nav2 /ros2_ws/src/coverage_geometry \
      --packages-up-to nav2 opennav_coverage \
      --build-base /lab/build --install-base /lab/install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DFields2Cover_DIR=/lab/build/fields2cover \
                   -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF -DBUILD_DOC=OFF -DBUILD_TUTORIALS=OFF
