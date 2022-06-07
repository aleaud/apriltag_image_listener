#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>

tf::TransformBroadcaster* br_ptr;
std::string odom_topic = "/odom";
std::string image_topic = "/image";
std::string odom_frame = "/odom";
std::string base_link_frame = "/base_link";
std::string camera_info_topic = "usb_cam/camera_info";
std::string camera_topic = "usb_cam/image_raw";
std::string camera_frame      = "";
ros::Publisher pub;

ros::Publisher* camera_info_publisher_ptr=0;

// Callback
void odomCallback(const nav_msgs::OdometryPtr& msg){
  geometry_msgs::Point& robot_position = msg->pose.pose.position;
  tf::Transform transform;
  const geometry_msgs::Point& position=msg->pose.pose.position;
  transform.setOrigin( tf::Vector3(position.x, position.y, position.z) );
  const geometry_msgs::Quaternion& orientation = msg->pose.pose.orientation;
  tf::Quaternion q;
  q.setX(orientation.x);
  q.setY(orientation.y);
  q.setZ(orientation.z);
  q.setW(orientation.w);
  transform.setRotation(q);
  br_ptr->sendTransform(tf::StampedTransform(transform, msg->header.stamp, odom_frame, base_link_frame));

  // if the camera frame has not been decided, we skip the transform
  if (camera_frame.length()==0) {
    std::cerr <<"waiting for first image to get the frame_id" << std::endl;
    return;
  }
  
  // camera forward looking
  Eigen::Matrix3f camera_orientation;
  camera_orientation <<
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0;
  Eigen::Quaternionf q_orientation(camera_orientation);
  transform.setOrigin(tf::Vector3(0.,0.,0.));
  q.setX(q_orientation.x());
  q.setY(q_orientation.y());
  q.setZ(q_orientation.z());
  q.setW(q_orientation.w());
  transform.setRotation(q);
  br_ptr->sendTransform(tf::StampedTransform(transform, msg->header.stamp, base_link_frame, camera_frame));
}

// Callback
void imageCallback(const sensor_msgs::ImagePtr& msg){
  // you may need to adjust the focal length
  float focal_lenght=267;

  camera_frame=msg->header.frame_id;
  sensor_msgs::CameraInfo camera_info;
  camera_info.header=msg->header;
  camera_info.width=msg->width;
  camera_info.height=msg->height;
  camera_info.K.fill(0);
  camera_info.K[0]=camera_info.K[4] = focal_lenght;
  camera_info.K[2]=camera_info.width/2;
  camera_info.K[8]=camera_info.height/2;
  camera_info_publisher_ptr->publish(camera_info);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "usb_cam_fixer_node");
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;
  br_ptr=&br;
  ros::Publisher  camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 1);
  camera_info_publisher_ptr=&camera_info_publisher;
  ros::Subscriber sub_odom = nh.subscribe(odom_topic,10,odomCallback);
  ros::Subscriber sub_image = nh.subscribe(camera_topic,10,imageCallback);
  ros::spin();
  return 0;
}
