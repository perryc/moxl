# Build stage
FROM ros:jazzy AS builder
ARG WORKSPACE=/opt/ws

SHELL ["/bin/bash", "-c"]

RUN mkdir -p $WORKSPACE
WORKDIR $WORKSPACE

# Copy dependency manifests first (cache layer)
COPY package.xml $WORKSPACE/
COPY src/toolpath_planner/package.xml $WORKSPACE/src/toolpath_planner/package.xml

# Install ROS dependencies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths . --ignore-src -r -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for toolpath planner
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-pyproj python3-shapely python3-numpy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy source
COPY . $WORKSPACE/

# Build both the main package and toolpath_planner
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --base-paths . src/toolpath_planner

# Runtime stage
FROM ros:jazzy
ARG USERNAME=moxl
ARG USER_UID=1001
ARG USER_GID=$USER_UID
ARG WORKSPACE=/opt/ws

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && usermod -aG dialout $USERNAME

COPY --from=builder $WORKSPACE/package.xml $WORKSPACE/
COPY --from=builder $WORKSPACE/src/toolpath_planner/package.xml $WORKSPACE/src/toolpath_planner/package.xml

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

# Install runtime dependencies
RUN apt-get update \
    && rosdep update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $WORKSPACE \
    && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths . \
    --ignore-src -r -y \
    --dependency-types=exec \
    && apt-get install -y --no-install-recommends python3-pyproj python3-shapely python3-numpy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy build artifacts
COPY --from=builder $WORKSPACE/install $WORKSPACE/install
COPY --from=builder $WORKSPACE/launch $WORKSPACE/launch
COPY --from=builder $WORKSPACE/config $WORKSPACE/config
COPY --from=builder $WORKSPACE/build $WORKSPACE/build
COPY --from=builder $WORKSPACE/description $WORKSPACE/description
COPY --from=builder $WORKSPACE/worlds $WORKSPACE/worlds
COPY --from=builder /opt/ros/$ROS_DISTRO /opt/ros/$ROS_DISTRO

RUN mkdir -p $WORKSPACE \
    && chown -R $USERNAME:$USERNAME $WORKSPACE

COPY utils/docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

USER $USERNAME
WORKDIR $WORKSPACE
ENV WORKSPACE=$WORKSPACE

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["ros2", "launch", "moxl", "sim.launch.py"]
