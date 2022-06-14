![header](https://capsule-render.vercel.app/api?type=rect&color=timeGradient&text=AGV%20SENSOR%20PACKAGE&fontSize=20)

## <div align=left>REPO INFO</div>  
- KAIST IRiS Lab. Autonomous Vehicle
- VEHICLE SENSOR PACKAGE  

## <div align=left>REPO CONFIG</div>  
#### PHAROS/MSG  
* ROS based VEHICLE COM MSG   
#### IMU  
* EB-IMU      : EB-IMU V5 9-DoF Low-Cost IMU SENSOR  
* Xsense-IMU  : XSENSE MTI-30 9-DOF AHRS IMU SENSOR  
#### LiDAR  
* MATLAB : MATLAB SUB SCRIPT
* RGB LiDAR : FUSING LiDAR + RGB VISION  
* Velodyne LiDAR : Velodyne VLP 16 Package with 3 LiDAR  
* Ouster LiDAR : Ouster OS-1 128 Package with 1 LiDAR  
#### TF  
* ROS TF : Transformation between Sensors  
#### VISION
* FLIR Blackfly S : FLIR BlackFly S USB STEREO VISION  
* UVC Logitech : UVC CAMERA for Logitech BRIO 4K 


## <div align=left>REPO USE</div> 
<pre>cd catkin_ws/  
git clone https://github.com/iismn/AGV-SENSOR-PACK  
cd .. && catkin_make</pre>

## <div align=left>ADD INFO</div>
#### VEHICLE CONSIST SENSOR 
- Ouster OS1-128 x1 [NEW]
- Velodyne VLP-16 Hi-Res LiDAR x3  
- Xsense MTI-30 AHRS IMU x1  
- EB-IMU V5 AHRS IMU x1  
- FLIR BlackFly S USB VISION x2  
- Logitech BRIO 4K USB VISION x1 
- Ublox F9P RTK-GPS x2 [NEW]
- Ublox M8P RTK-GPS x1
- <del>Novatel RTK GPS+INS x1</del>
- <del>Velodyne HDL-32 Hi-Res LiDAR x1</del>

#### V 1.2.0
- Add Ouster OS1-128
- Update Velodyne LiDAR driver
- Ouster LiDAR driver composed with ROS-Time (Packet / ROS Message)
- Ublox F9P add (On Going)
