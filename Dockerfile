FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="qianli.ma622@gmail.com"
LABEL version="0.1"
LABEL description="This is a customize Docker image for running highway roadmap project"

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update

RUN apt install -y ninja-build python3-pip cmake gcc-9 g++-9 && \
    apt install -y git vim powerline fonts-powerline clang-tidy clang-format clang-tidy-12 clang-format-12 && \
    apt install -y libeigen3-dev libgmp-dev libompl-dev ompl-demos liburdfdom-dev libcgal-dev && \
    apt install -y libtinyxml2-dev && \
    pip3 install powerline-status powerline-shell && \
    pip install conan && \
    apt clean

# add robot as a non-root user for the system account
ARG USERNAME=robot
# ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN useradd -ms /bin/bash $USERNAME

RUN apt update && \
    apt install -y sudo && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/robot

COPY script/install-dependencies-for-docker-image.sh /
RUN /install-dependencies-for-docker-image.sh

# According to the github action documentation, one should run all actions in containers as the root user
USER root

# User powerline shell for terminal beautification
RUN echo '\n\
function _update_ps1() {\n\
        PS1=$(powerline-shell $?) \n\
}\n\
\n\
if [[ $TERM != linux && ! $PROMPT_COMMAND =~ _update_ps1  ]]; then\n\
        PROMPT_COMMAND="_update_ps1; $PROMPT_COMMAND"\n\
fi\n\
' >> /home/robot/.bashrc
