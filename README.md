# Kinect Client on Ubuntu - Color, Depth, Infrared, Body Tracking, Face Tracking


1) git clone https://github.com/sdebnathusc/kinect_bridge_ubuntu.git

2) Open terminal. 

3) cd kinect_bridge_ubuntu/ 
 
4) Run the below command: 
 
  sudo docker run -it --rm --net=host --name kinectbridgecontainer  -v "$PWD/:/catkin_ws/src/kinect-bridge2" shoubhikdn/kinect-bridge /bin/bash 
 
  This assumes that docker is installed. It will make a local copy of the image shoubhikdn/kinectbridge in the container named kinectbridgecontainer. 
 
5) cd catkin_ws/ 

6) . devel/setup.bash 

7) export ROS_IP=127.0.0.1   

8) catkin_make 

9) roslaunch kinect_bridge2 kinect_client.launch server_ip:=<ip_of_windows_machine> 

This will start up Kinect Client. 
