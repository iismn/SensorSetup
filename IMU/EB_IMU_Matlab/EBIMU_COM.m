%% Clear Work Space
clc; clear;

%% EB-IMU 9DoF Initialize
EBIMU = serialport("/dev/ttyUSB0",115200);

%% EB-IMU : Unload Broken Data
for i = 1:10
    Raw_DATA = readline(EBIMU);
end

%% EB-IMU : Sensor Data Parse
iter = 0;
while true
    
    Raw_DATA = readline(EBIMU);
    Raw_DATA = extractAfter(Raw_DATA,"*");
    Raw_DATA = split(Raw_DATA,",");
    
    IMU.Quaternion.w = str2double(Raw_DATA(4));
    IMU.Quaternion.x = str2double(Raw_DATA(3));
    IMU.Quaternion.y = str2double(Raw_DATA(2));
    IMU.Quaternion.z = str2double(Raw_DATA(1));
    
    RPY = quat2eul([IMU.Quaternion.w IMU.Quaternion.x IMU.Quaternion.y IMU.Quaternion.z],"ZYX");
    
    IMU.RPY.Roll = rad2deg(RPY(3));
    IMU.RPY.Pitch =  rad2deg(RPY(2));
    IMU.RPY.Yaw = rad2deg(RPY(1));

    IMU.Gyro.x = str2double(Raw_DATA(5));
    IMU.Gyro.y = str2double(Raw_DATA(6));
    IMU.Gyro.z =  str2double(Raw_DATA(7));
    
    IMU.Accel.x = str2double(Raw_DATA(8))*9.81;
    IMU.Accel.y = str2double(Raw_DATA(9))*9.81;
    IMU.Accel.z = str2double(Raw_DATA(10))*9.81;
   
    [IMU.RPY.Roll, IMU.RPY.Pitch, IMU.RPY.Yaw];
    iter = iter+1;
    
    figure(1)
    xlim([-2 2])
    ylim([-2 2])
    zlim([-2 2])    
    ax = plotTransforms([0 0 0],[IMU.Quaternion.w IMU.Quaternion.x IMU.Quaternion.y IMU.Quaternion.z],'FrameSize',5);

    hold on
    axis equal
    grid on
    hold off
    
end

