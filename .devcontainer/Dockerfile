FROM stereolabs/zed:5.1.2-gl-devel-cuda12.8-ubuntu22.04

RUN apt-get update && apt-get upgrade -y
# Install important packages
RUN apt-get install -y \
    git \
    curl \
    usbutils

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_DISTRO=jazzy

RUN echo "Setup User: $USERNAME (UID:$USER_UID / GID:$USER_GID)" && \
    # Handle the GROUP
    if getent group ${USER_GID} > /dev/null; then \
        echo "Group ${USER_GID} exists. Renaming it to ${USERNAME}."; \
        groupmod -n ${USERNAME} $(getent group ${USER_GID} | cut -d: -f1); \
    else \
        echo "Group ${USER_GID} does not exist. Creating it."; \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi && \
    # Handle the USER
    if getent passwd ${USER_UID} > /dev/null; then \
        echo "User ${USER_UID} exists. Renaming it to ${USERNAME}."; \
        EXISTING_USER=$(getent passwd ${USER_UID} | cut -d: -f1); \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m ${EXISTING_USER}; \
        # Ensure the user belongs to the correct primary group
        usermod -g ${USER_GID} ${USERNAME}; \
    else \
        echo "User ${USER_UID} does not exist. Creating it."; \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}; \
    fi && \
    # Add sudo support
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

ENV SHELL=/bin/bash

# Install Pixi
RUN curl -fsSL https://pixi.sh/install.sh | PIXI_HOME=/usr/local/bin/pixi bash

# Switch from root to user
USER $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install pixi-ros plugin
RUN /usr/local/bin/pixi/bin/pixi global install pixi-ros

# Add Pixi to PATH
RUN echo "export PATH=\"/usr/local/bin/pixi/bin:/home/$USERNAME/.pixi/bin:\$PATH\"" >> ~/.bashrc