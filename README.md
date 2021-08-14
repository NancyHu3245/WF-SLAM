# WF-SLAM
## This project base on those repositories below:

Core project1: 
    <b>https://github.com/raulmur/ORB_SLAM2</b>

Core project2: 
    <b>https://github.com/BertaBescos/DynaSLAM</b>

## Building
cd catkin_workspace/src  　　
git https://github.com/NancyHu3245/WF-SLAM.git  　　
cd ../  　　
./build.sh  　　


## Usage
Configure the following four .json files in json-config in vscode, change the relevant file path according to your file path, and then press F5 to run.


## TUM Dataset
Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.  

![image](https://github.com/NancyHu3245/WF-SLAM/blob/main/pic1.png)

![image](https://github.com/NancyHu3245/WF-SLAM/blob/main/wf-slam1.png)

![image](https://github.com/NancyHu3245/WF-SLAM/blob/main/pic2.png)

![image](https://github.com/NancyHu3245/WF-SLAM/blob/main/wf-slam2.png)




## Real World 
Only implement RGBD(ROS / ZED camera) example:
![image](https://github.com/NancyHu3245/WF-SLAM/blob/main/zed.png)
![image](https://github.com/NancyHu3245/WF-SLAM/blob/main/zed2.png)

