/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

//library
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <stdint.h>
#include <iostream>

serial::Serial ser;
std::string setPortName("/dev/ttyUSB0");
int BaudrateNum(115200);
int hzNum_I(100);
double hzNum_D(100);

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port : " << std::endl << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("setPort", setPortName, "/dev/ttyUSB0");
    pnh.param<int>("setBaudrate", BaudrateNum, 115200);
    pnh.param<int>("setHz", hzNum_I, 100);

    ros::Subscriber write_sub = nh.subscribe("settingValue", 1, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/pharos/imu/serial", 1);
    ros::Publisher okPub = nh.advertise<std_msgs::String>("okTopic", 1);

    hzNum_D = (double)hzNum_I;

    try
    {
        ser.setPort(setPortName);
        ser.setBaudrate(BaudrateNum);
        serial::Timeout to = serial::Timeout::simpleTimeout(hzNum_I);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }


    if(ser.isOpen())
    {
        ROS_INFO_STREAM("[PHAROS] SENSOR IMU : Serial Port INIT OK");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(hzNum_D);

    while(ros::ok())
    {
        ros::spinOnce();

        if(ser.available())
        {
            std_msgs::String result;
            result.data = ser.read(ser.available());
            read_pub.publish(result);
            if(result.data == "<ok>")
            {
                okPub.publish(result);
            }
        }
        loop_rate.sleep();
    }

}
