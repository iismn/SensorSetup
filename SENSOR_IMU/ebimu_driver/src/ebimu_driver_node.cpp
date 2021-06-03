#define GRAVITY 9.81

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <serial/serial.h>

#include <pharos_msgs/EBIMUheader.h>

#include <stdio.h>
#include <stdint.h>
#include <iostream>

// PARAMS SETTING
serial::Serial ser;
std::string str="";
char c = ' ';
int comma[12];
std::string strCompare0 = "<ok>";
std::string strCompare1 = "*";
std::string cmd_reset = "\n";
std::string cmd_offset = "\n";
std::string cmd_s0, cmd_s1 = "\n";
std::string R_s, P_s, Y_s, Acc_x, Acc_y, Acc_z, Gyro_x, Gyro_y, Gyro_z, Mag_x, Mag_y, Mag_z;

// SENSOR MESSAGE SETTING
pharos_msgs::EBIMUheader imu_pharos;
sensor_msgs::Imu imu_ros;

// SERIAL SETTING
std::string setPortName("/dev/ttyUSB0");
int BaudrateNum(115200);
double hzNum(100);

// SER WRITE -------------------------------------------------------------
void write_callback(const std_msgs::String::ConstPtr& msg){
    ser.write(msg->data);
}

// MAIN ------------------------------------------------------------------
int main (int argc, char** argv){
    ros::init(argc, argv, "ebimu_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("setPort", setPortName, "/dev/ttyUSB0");
    pnh.param<int>("setBaudrate", BaudrateNum, 115200);
    pnh.param<double>("setHz", hzNum, 1000);

    pnh.param<std::string>("reset", cmd_reset, "<reset>");
    pnh.param<std::string>("cmo", cmd_offset, "<cmo>");
    pnh.param<std::string>("frame_id_pharos", imu_pharos.header.frame_id, "ebimu");
    pnh.param<std::string>("frame_id_ros", imu_pharos.header.frame_id, "imu_link");
    ros::Subscriber write_sub = nh.subscribe("/pharos/imu/write", 1, write_callback);
    ros::Publisher pub_imu_pharos = nh.advertise<pharos_msgs::EBIMUheader>("/pharos/imu/raw", 1);
    ros::Publisher pub_imu_ros = nh.advertise<sensor_msgs::Imu>("/pharos/ros/imu/raw", 1);

    // [PHAROS] INERTIAL MEAS. UNIT : SERIAL COM CHECK
    try
    {
        ser.setPort(setPortName);
        ser.setBaudrate(BaudrateNum);
        serial::Timeout to = serial::Timeout::simpleTimeout(hzNum);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("[PHAROS] SENSOR IMU : Serial Port DISCONNECTED ");
        return -1;
    }

    // [PHAROS] INERTIAL MEAS. UNIT : SERIAL INIT
    if(ser.isOpen())
    {
        std::cout << "[PHAROS] SENSOR IMU : Serial Port     / [INIT OK]" << std::endl;
        std::cout << "[PHAROS] SENSOR IMU : Serial Port     / [" << setPortName << "]"<< std::endl;
        std::cout << "[PHAROS] SENSOR IMU : Serial Baudrate / [" << BaudrateNum << "]"<< std::endl;
        std::cout << "[PHAROS] SENSOR IMU : Serial Hz       / [" << hzNum << "]"<< std::endl;
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(hzNum);

    pnh.getParam("reset", cmd_reset);
    if( cmd_reset.compare(cmd_s0) != 0){
        ser.write(cmd_reset + "\n");
        std::cout << "[PHAROS] SENSOR IMU : Serial IMU      / [" << cmd_reset << "]" << std::endl;
        cmd_s0 = cmd_reset;
    }
    ros::Duration(2).sleep();
    // pnh.getParam("cmo", cmd_offset);
    // if( cmd_offset.compare(cmd_s1) != 0){
    //     ser.write(cmd_offset + "\n");
    //     std::cout << "[PHAROS] SENSOR IMU : Serial IMU      / [" << cmd_offset << "]" << std::endl;
    //     cmd_s1 = cmd_offset;
    // }

    while(ros::ok())
    {
        ros::spinOnce();

        while(ser.available())
        {
            std::string s = ser.read();
            c = *s.c_str();
            if( c == '\n'){
                    if(strCompare0.compare( str.substr(0, 4) ) == 0 ){
                        std::cout<<str.substr(0, 4)<<std::endl;
                        str = str.substr(5, str.length());
                    }
                    if(strCompare1.compare( str.substr(0, 1) ) == 0 ){
                        comma[0] = str.find("*");
                        for(int i=1; i<12; i++){
                            comma[i] = str.find(",", comma[i-1]+1);
                        }
                        // IMU PHAROS MESSAGE - PERM RAW DATA
                        R_s = str.substr(comma[0]+1, comma[1]-comma[0]);
                        P_s = str.substr(comma[1]+1, comma[2]-comma[1]);
                        Y_s = str.substr(comma[2]+1, comma[3]-comma[2]);

                        Gyro_x = str.substr(comma[3]+1, comma[4]-comma[3]);
                        Gyro_y = str.substr(comma[4]+1, comma[5]-comma[4]);
                        Gyro_z = str.substr(comma[5]+1, comma[6]-comma[5]);

                        Acc_x = str.substr(comma[6]+1, comma[7]-comma[6]);
                        Acc_y = str.substr(comma[7]+1, comma[8]-comma[7]);
                        Acc_z = str.substr(comma[8]+1, comma[9]-comma[8]);

                        Mag_x = str.substr(comma[9]+1, comma[10]-comma[9]);
                        Mag_y = str.substr(comma[10]+1, comma[11]-comma[10]);
                        Mag_z = str.substr(comma[11]+1, str.length()-comma[11]);

                        // * RPY RESULT *
                        // "PHAROS" Type Msg (ROLL PITCH YAW)
                        imu_pharos.ebimu.RPY.x = atof(R_s.c_str());
                        imu_pharos.ebimu.RPY.y = -atof(P_s.c_str());
                        imu_pharos.ebimu.RPY.z = -atof(Y_s.c_str());
                        // "ROS" Type Msg (QUATERNION)
                        tf::Quaternion QUAT_IMU;
                        QUAT_IMU.setRPY(atof(R_s.c_str())*(M_PI / 180.0),-atof(P_s.c_str())*(M_PI / 180.0),-atof(Y_s.c_str())*(M_PI / 180.0));
                        QUAT_IMU.normalize();

                        imu_ros.orientation.w = QUAT_IMU.w();
                        imu_ros.orientation.x = QUAT_IMU.x();
                        imu_ros.orientation.y = QUAT_IMU.y();
                        imu_ros.orientation.z = QUAT_IMU.z();

                        // * GYRO SCOPE *
                        // "PHAROS" Type Msg
                        imu_pharos.ebimu.GYR.x = atof(Gyro_x.c_str());
                        imu_pharos.ebimu.GYR.y = atof(Gyro_y.c_str());
                        imu_pharos.ebimu.GYR.z = atof(Gyro_z.c_str());
                        // "ROS" Type Msg
                        imu_ros.angular_velocity.x = atof(Gyro_x.c_str())*(M_PI / 180.0);
                        imu_ros.angular_velocity.y = atof(Gyro_y.c_str())*(M_PI / 180.0);
                        imu_ros.angular_velocity.z = atof(Gyro_z.c_str())*(M_PI / 180.0);

                        // * ACCEL SCOPE *
                        // "PHAROS" Type Msg
                        imu_pharos.ebimu.ACC.x = atof(Acc_x.c_str()) * GRAVITY;
                        imu_pharos.ebimu.ACC.y = atof(Acc_y.c_str()) * GRAVITY;
                        imu_pharos.ebimu.ACC.z = atof(Acc_z.c_str()) * GRAVITY;
                        // "ROS" Type Msg
                        imu_ros.linear_acceleration.x = atof(Acc_x.c_str()) * GRAVITY;
                        imu_ros.linear_acceleration.y = atof(Acc_y.c_str()) * GRAVITY;
                        imu_ros.linear_acceleration.z = atof(Acc_z.c_str()) * GRAVITY;

                        // * MAGNET SCOPE *
                        // "PHAROS" Type Msg
                        imu_pharos.ebimu.MAG.x = atof(Mag_x.c_str());
                        imu_pharos.ebimu.MAG.y = atof(Mag_y.c_str());
                        imu_pharos.ebimu.MAG.z = atof(Mag_z.c_str());

                        imu_pharos.header.stamp = ros::Time::now();
                        imu_ros.header.stamp = ros::Time::now();

                        pub_imu_pharos.publish(imu_pharos);
                        pub_imu_ros.publish(imu_ros);
                    }
                // }
                str = "";
            }else{
                str.push_back(c);
            }
        }
        loop_rate.sleep();
    }

}
