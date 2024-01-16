# Amazon-Grasping-Project
This repository consists the vision, motion planning and tactile sensing scripts for the Amazon Grasping Project
# Vision
This folder contains script that identifies the grasping location of objects. The script uses feature matching algorithm to match the target object and searches the observation space for the target object. Once the enough matching features are found, a bouding box is drawn around the object. Then the coordinates of the object are transformed to world coorinates followed by another transformation which gives the coordinates of the object with respect to the end effector of the UR5e. <br /> 
<br /> 
Before cloning this script into the repo, clone and follow the instructions in `https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy` to ensure that the RealSense wrapper has been correctly installed.
<br />
Now you can clone this script into the src directory of the realsense2_camera project.
<br />

The script takes the target image labelled "1.jpg" and scans the camera feed for the target image which is found using SIFT feature matching. This script also contains a rostopic publisher, with the name "Location" with a publisher with name "LocationPublisher". The data is outputted as a String in the format "x: X.XX, y: Y.YY, z: Z.ZZ", all the numbers are in meters. The outputted numbers are the distances of the object from the end-effector of the robot. <br/>
The image shows the script detecting the object and the publisher outputting the data.
<br/>
![image](https://user-images.githubusercontent.com/92841422/215548890-e7adca09-ac4f-4580-b5c8-a0231a35bb85.png)
<br />
<br />
You can run the script by
<br/>
`rosrun realsense2_camera RecogSIFT.py`
# Tactile Sensing
This folder contains the classifier that predicts if an object is slipping from the grasp of the robot. This also contains the script tracking_modified.py which uses the classifier_SVM.sav to predicts if an object is slipping in real-time using the output from the GelSight sensor.
Before doing this, please clone and follow the instructions given by the GelSight Github repo here: 'https://github.com/gelsightinc/gsrobotics'. Place the files from the folder in the directory gsrobotics/demo/min_tracking_linux_v0. Upon running the tracking_modified.py you should see the following output:

![Screenshot from 2023-05-18 12-07-06](https://github.com/apravenkat/Amazon-Grasping-Project/assets/92841422/02ff7d06-d44f-46b5-9ed1-1beb77d35042)

If you are using the UR Controller PC, this script is called modified_tracking.py located in the following directory: /home/macs/tracking/gsrobotics-main/demos/mini_marker_tracking. All the parameters for the camera have already been configured.
