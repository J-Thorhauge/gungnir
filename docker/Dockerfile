#   ________                            .__         #
#  /  _____/ __ __  ____    ____   ____ |__|______  #
# /   \  ___|  |  \/    \  / ___\ /    \|  \_  __ \ #
# \    \_\  \  |  /   |  \/ /_/  >   |  \  ||  | \/ #
#  \______  /____/|___|  /\___  /|___|  /__||__|    #
#         \/           \//_____/      \/            #

FROM ros:humble

# Setup user
ARG USERNAME=gungnir
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

####################
##### Installs #####
####################

# Avoid prompts with apt-get
ARG DEBIAN_FRONTEND=noninteractive

# Install Utilities
RUN apt-get update && apt-get install -y \
  nano \
  net-tools

# Install Interfaces
RUN apt-get update && apt-get install -y python3-pip && \
  pip3 install \
  canopen

# Install Zenoh for better ros2 network performance
RUN apt-get update && apt-get install -y ros-humble-rmw-zenoh-cpp

# Clean up
RUN rm -rf /var/lib/apt/lists/*

#####################################
##### Copy and set up workspace #####
#####################################

# Create workspace directory
RUN mkdir -p /home/ws

# Copy the source code
COPY ./src /home/ws/src

# Set working directory
WORKDIR /home/ws

# Install dependencies for the ros packages
# RUN /bin/bash -c "apt-get update && \
#     source /opt/ros/humble/setup.bash && \
#     source install/setup.bash && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y --rosdistro humble"

# Build the ros packages
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
#     source install/setup.bash && \
#     colcon build

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Enjoy ;)