<!-- GETTING STARTED -->
## Overview
This repo contains the code for unitree h1_2 containarized with docker. The submodule cufino_ws_h1_2 is stand-alone and can be built also without docker. However, it is strongly suggest to follow this guide to have a containarized application. The code consists in four main folders:\
- [h1_2_motion](https://github.com/francescocufino/cufino_ws_h1_2/tree/main/h1_2_motion) contains the implementation of basic motions (arm, hands, locomotion) exploiting high level functionalitis of unitree_sdk2;
- [h1_2_demo](https://github.com/francescocufino/cufino_ws_h1_2/tree/main/h1_2_demo) contains an example user program based on h1_2_motion to perform whole body motion;
- [h1_2_pushing](https://github.com/francescocufino/cufino_ws_h1_2/tree/main/h1_2_pushing) contains the code under development for the wheelchair pushing project;
- [h1_2_description](https://github.com/francescocufino/cufino_ws_h1_2/tree/main/h1_2_description) contains a universal humanoid robot description (URDF & MJCF) for the [Unitree H1_2](https://www.unitree.com/h1), developed by [Unitree Robotics](https://www.unitree.com/).


## Prerequisites
Before setting up the project, you have to install docker. If you already installed docker, go to the next session
```sh
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install the Docker packages
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add docker user
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```
Then log out and log in.
## Compilation

1. Clone the repo
```sh
git clone --recurse-submodules https://github.com/francescocufino/unitree_h1_2.git
```
2.  Open a terminal in the repo and source the build file
```sh
source docker_build.sh
```

## Subfolders
From now, please refer to [README.MD](https://github.com/francescocufino/cufino_ws_h1_2/tree/main/README.md)


## Development
Once the container is running, you can develop from inside it using VsCode. Open VsCode from your host machine, install the extension Dev Containers, and in command bar select attach to running container. Open the src folder of the project you want to develop. 

The submodule cufino_ws is also uploaded on the PC2 on the robot. If you are on the same network of the robot, you can directly connect to it and develop from the PC2 through SSH. In this case, you can use the extension of vscode SSH. (Actually blocked)
   
   
   
   
   
   
   
   
   
   
   
   
   
   

