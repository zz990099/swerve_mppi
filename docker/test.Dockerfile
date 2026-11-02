FROM ros:jazzy-ros-base-noble

LABEL org.opencontainers.image.source="https://github.com/zz990099/swerve_mppi" \
      org.opencontainers.image.description="MPPI local testing: ROS 2 Jazzy, Gazebo Harmonic, pinned chassis and built Nav2 plugins"

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-colcon-common-extensions \
      python3-psutil python3-yaml python3-matplotlib python3-pytest \
      ros-jazzy-ament-cmake-gtest ros-jazzy-ament-cmake-pytest \
      ros-jazzy-ament-cmake-python ros-jazzy-rosidl-default-generators \
      ros-jazzy-tf2-geometry-msgs ros-jazzy-nav2-controller \
      ros-jazzy-nav2-lifecycle-manager ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control ros-jazzy-joint-state-broadcaster \
      ros-jazzy-forward-command-controller ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/mppi_ws
COPY swerve_mppi_core src/swerve_mppi/swerve_mppi_core
COPY swerve_mppi_ros src/swerve_mppi/swerve_mppi_ros
COPY companion-chassis src/swerve_gazebo_sim
ARG MPPI_REVISION
ARG CHASSIS_REVISION
LABEL org.opencontainers.image.revision="${MPPI_REVISION}" \
      io.github.swerve_mppi.chassis-revision="${CHASSIS_REVISION}"
# The recording runner reads this value instead of requiring a .git directory.
ENV MPPI_REVISION=${MPPI_REVISION} \
    GITHUB_SHA=${MPPI_REVISION} \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    ROS_DOMAIN_ID=83 \
    MPLBACKEND=Agg
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --base-paths src --packages-up-to swerve_mppi_ros \
      --parallel-workers 2 --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Werror \
                   -DCMAKE_PREFIX_PATH=/opt/mppi_ws/install/swerve_chassis_core \
                   -DBUILD_TESTING=ON
COPY docker/test-entrypoint.sh /mppi-entrypoint.sh
RUN chmod +x /mppi-entrypoint.sh
WORKDIR /opt/mppi_ws/src/swerve_mppi
ENTRYPOINT ["/mppi-entrypoint.sh"]
CMD ["bash"]
