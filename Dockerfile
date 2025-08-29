FROM ubuntu:20.04

#Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0

#Install essential
RUN apt-get update && apt-get install -y build-essential libssl-dev wget sudo && apt-get -y install cmake protobuf-compiler && apt install -y python3-pip  
  
#Install other dependencies
RUN apt-get install -y libeigen3-dev libboost-all-dev libspdlog-dev
RUN apt-get install -y liborocos-kdl-dev libkdl-parser-dev
RUN apt install -y python3-venv
RUN apt-get clean 

#Add non root user using UID and GID passed as argument
ENV HOME /home/user
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
RUN echo "user:user" | chpasswd
RUN echo "user ALL=(ALL:ALL) ALL" >> /etc/sudoers
USER user

#Install ACADOS
COPY --chown=user acados ${HOME}/acados
#COPY dependencies/acados ${HOME}/acados
RUN mkdir ${HOME}/acados/build
WORKDIR ${HOME}/acados/build
RUN cmake -DACADOS_WITH_QPOASES=ON ..
RUN make install -j4
RUN echo "source ${HOME}/acados/acados_env.sh" >> ${HOME}/.bashrc

COPY --chown=user /unitree_sdk2 ${HOME}/unitree_sdk2
COPY --chown=user /unitree_sdk2_python ${HOME}/unitree_sdk2_python

#Python venv
RUN python3 -m venv ${HOME}/venv
RUN chmod -R 777 ${HOME}/venv
RUN ${HOME}/venv/bin/pip3 install --upgrade pip

#Install unitree_sdk2_python
WORKDIR ${HOME}/unitree_sdk2_python
RUN ${HOME}/venv/bin/pip3 install -e .


#Install inspire_hand_sdk
COPY --chown=user /inspire_hand_sdk ${HOME}/inspire_hand_sdk
WORKDIR ${HOME}/inspire_hand_sdk
#RUN pip install --upgrade pip
USER root
RUN apt install -y python3-pyqt5
USER user
RUN ${HOME}/venv/bin/pip3 install -e .
ENV PATH="${HOME}/venv/bin:$PATH"


#Compile unitree_skd2
WORKDIR ${HOME}/unitree_sdk2
RUN mkdir build
WORKDIR ${HOME}/unitree_sdk2/build
RUN cmake ..
USER root
RUN make install
USER user
RUN make

COPY --chown=user /cufino_ws_h1_2 ${HOME}/cufino_ws_h1_2


#Compile cufino_ws
WORKDIR ${HOME}/cufino_ws_h1_2/
RUN mkdir build
WORKDIR ${HOME}/cufino_ws_h1_2/build
RUN cmake ..
RUN make


#Clean image
USER root
RUN rm -rf /var/lib/apt/lists/*
USER user

#Set non root user and home work directory

WORKDIR ${HOME}




