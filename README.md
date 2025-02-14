<!-- GETTING STARTED -->
## Overview
This repo contains the code for unitree h1_2. 


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
## Compilation and execution

1. Clone the repo (complete the command)
```sh
git clone https://github.com/francescocufino/unitree_h1.git
```
2.  Open a terminal in the repo and source the build file
```sh
source docker_build.sh <IMAGE_NAME>
```

3. In the terminal in the repo, source the run file
```sh
source docker_run.sh <IMAGE_NAME> <CONTAINER_NAME>
```

4. When you want to close the application, type
```sh
exit
docker stop <CONTAINER_ID>
xhost -local:root
```

5. If you want to start again the application,
```sh
docker start <CONTAINER_ID>
docker exec -it <CONTAINER_ID> bash
```

## Development
Once the container is running, you can develop from inside it using VsCode. Open VsCode from your host machine, install the extension Dev Containers, and in command bar select attach to running container. Open the folder /home/user/ros2_ws/src, which is the binded folder. 

The submodule cufino_ws is also uploaded on the PC2 on the robot. If you are on the same network of the robot, you can directly connect to it and develop from the PC2 through SSH. In this case, you can use the extension of vscode SSH.

##Connection to robot PC2
To connect to the robot PC2 through ssh you can do the following procedure:

1. Ensure that the robot PC2 is turned on and the wi fi adapter is connected. Then, it automatically should connect to the H1_unitree network. Connect to the network
SSID: H1_unitree
pwd: Unitree0408


2. Connect with ssh. The IP address of H1 PC2 is 192.168.2.10, assigned statically by the router.
If it is not for some reason, you can easily obtain it through
```sh
sudo nmap -T4 -sP 192.168.2.0/24 | grep -B 2 "E4:FA:C4:4C:B2:F8"
```
Then connect
```sh
ssh unitree@192.168.2.10
```
Password Unitree0408.
Now you are connected.

3. If the robot PC2 is not shown and you are not able to connect, either it did not turned on or you should reconnect to the network. In this case, connect through ethernet (the port is the same of low level pc), assign to your machine a static address in the subnet 192.168.123.0/24, like 192.168.123.222, and run
```sh
ssh unitree@192.168.123.162
```
Password Unitree0408.
Now you are connected.

If you want to reconnect the robot to the wi fi network in such way to avoid using the cable, type
```sh
sudo nmcli dev wifi connect "H1_unitree" password "Unitree0408"
```


   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

