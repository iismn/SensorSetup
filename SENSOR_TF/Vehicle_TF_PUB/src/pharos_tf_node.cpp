
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <Eigen/Geometry>

#include <pharos_msgs/SCBADUheader.h>
#include <pharos_msgs/CAN_GWAY_header.h>
#include <pharos_msgs/StateStamped2016.h>

ros::Time bag_stamp;
bool bag_;
int rate_;
int tf_num_;

typedef struct tf_param_{
    std::string frame_id;
    std::string child_frame_id;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    tf::Transform transform;
}tf_param_;

void UpdateTF(struct tf_param_ &tf_param){
    tf_param.transform.setOrigin(tf::Vector3(tf_param.x, tf_param.y, tf_param.z));
    tf::Quaternion tf_RM;
    tf_RM.setRPY(tf_param.roll * M_PI/180, tf_param.pitch * M_PI/180, tf_param.yaw * M_PI/180);
    tf_param.transform.setRotation(tf_RM);
}

void TimeStampIMGCb(const sensor_msgs::ImageConstPtr& img_msg){
  bag_stamp = img_msg->header.stamp;
}

void TimeStampIMUCb(const sensor_msgs::Imu::ConstPtr& imu_raw){
  bag_stamp = imu_raw->header.stamp;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "vhicle_tf_broadcaster");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");


    pnh.param("bag", bag_, false);
    pnh.param("rate", rate_, 500);
    pnh.getParam("tf_num", tf_num_);

    ros::Rate rate(rate_);
    ros::Subscriber sub = nh.subscribe("/imu/data", 1, TimeStampIMUCb);

    static tf::TransformBroadcaster br;
    tf_param_ *tf_param = new tf_param_[tf_num_];
    tf_param_ tf_0;
    for(int i=0; i<tf_num_; i++){
      std::stringstream tf_name;
      tf_name << "tf_" << i+1;
      pnh.getParam(tf_name.str()+"/frame_id", tf_param[i].frame_id);
      pnh.getParam(tf_name.str()+"/child_frame_id", tf_param[i].child_frame_id);
      pnh.getParam(tf_name.str()+"/x", tf_param[i].x);
      pnh.getParam(tf_name.str()+"/y", tf_param[i].y);
      pnh.getParam(tf_name.str()+"/z", tf_param[i].z);
      pnh.getParam(tf_name.str()+"/roll", tf_param[i].roll);
      pnh.getParam(tf_name.str()+"/pitch", tf_param[i].pitch);
      pnh.getParam(tf_name.str()+"/yaw", tf_param[i].yaw);
      UpdateTF(tf_param[i]);
    }

    ros::Time stamp;
    while(ros::ok()){

        if(bag_){
          stamp = bag_stamp;
        }
        else {
          stamp = ros::Time::now();
        }

        // Publish fixed tf
        for(int i=0; i<tf_num_; i++){
            br.sendTransform(tf::StampedTransform(tf_param[i].transform, stamp, tf_param[i].frame_id, tf_param[i].child_frame_id));
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
