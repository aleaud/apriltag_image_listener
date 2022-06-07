#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <Eigen/Geometry>

using namespace std;

//global variables
ros::Publisher img_pub, camera_info_pub;
const int ROBOT_POSE_ID_LIMIT = 1153;									
int robot_pose_node_id = 1101;
Eigen::Vector2f tag_transforms[14][100];							// store all the detected transformed wrt robot
bool is_detected[14];												// keep track of all the tag nodes already stored
int amount_detections[14];											// contains how many detections has been collected for each tag
bool stop = false;

//robot transform elements in 2D at time t
geometry_msgs::Pose2D curr_robot_pose;
//robot transform elements in 2D at time t-1
geometry_msgs::Pose2D prev_robot_pose;		


//labels for g2o file
const string label_VERTEX_XY("VERTEX_XY");							//position of landmark in robot frame 
const string label_VERTEX_SE2("VERTEX_SE2");						//position of robot
const string label_EDGE_SE2_XY("EDGE_SE2_XY");						//edge that links a robot pose and a tag pose
const string label_EDGE_SE2("EDGE_SE2");							//edge between two robot poses

/*	
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msgScan) 
{ // DON'T NEED SCAN CALLBACK, BUT IT CANNOT WORKS IF DISAPPEARS 
} */

//maybe it can be deleted
void captureImageCallback(const sensor_msgs::ImagePtr& img) {
	//send images and camera info to apriltag_ros node
	sensor_msgs::CameraInfo msg_info;
	msg_info.header.seq = img->header.seq;
	msg_info.header.stamp = img->header.stamp;
	msg_info.header.frame_id = img->header.frame_id; 
	msg_info.height = img->height;
	msg_info.width = img->width;
	msg_info.distortion_model = "plumb_bob";
	//D
	msg_info.D = {0.0,0.0,0.0,0.0,0.0};
	//K
	msg_info.K[0] = 269.852539;			//fx	
	//msg_info.K[1] = 0.0;
	msg_info.K[2] = 157.051246;			//cx
	//msg_info.K[3] = 0.0;
	msg_info.K[4] = 269.733262;			//fy
	msg_info.K[5] = 113.1178;			//cy
	//msg_info.K[6] = 0.0;
	//msg_info.K[7] = 0.0;
	//msg_info.K[8] = 1.0;
	//R
	msg_info.R[0] = 1.0;
	msg_info.R[1] = 0.0;
	msg_info.R[2] = 0.0;
	msg_info.R[3] = 0.0;
	msg_info.R[4] = 1.0;
	msg_info.R[5] = 0.0;
	msg_info.R[6] = 0.0;
	msg_info.R[7] = 0.0;
	msg_info.R[8] = 1.0;
	//P
	msg_info.P[0] = 262.5;				//fx'
	//msg_info.P[1] = 0.0;
	msg_info.P[2] = 159.5;				//cx'
	msg_info.P[3] = 0.0;				//Tx
	//msg_info.P[4] = 0.0;
	msg_info.P[5] = 262.5;				//fy'
	msg_info.P[6] = 119.5;				//cy'
	msg_info.P[7] = 0.0;				//Tx'	
	//msg_info.P[8] = 0.0;
	//msg_info.P[9] = 0.0;
	//msg_info.P[10] = 1.0;
	//msg_info.P[11] = 0.0;
	
	//binning: default settings
	//msg_info.binning_x = 0;
	//msg_info.binning_y = 0;
	
	//roi: default settings (all zeros)

	//publish messages throught AprilTag topics
	camera_info_pub.publish(msg_info);
	img_pub.publish(img);
} 


void detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& a) {
	//NOTE: there are no tag bundles in the bag!
	for(int i=0; i < a->detections.size(); i++) {
		if(i > 0) return;							//ignore bundle-like detections
		//show data
		apriltag_ros::AprilTagDetection det = a->detections[i];
		int id = det.id[i];
		//cout << "Tag ID -> " << id << ", size -> " << det.size[i] << endl;
		geometry_msgs::PoseWithCovarianceStamped& p = det.pose;
		float x = p.pose.pose.position.x, y = p.pose.pose.position.y, z = p.pose.pose.position.z;
		//cout << "Pose info: pose.x -> " << x << ", pose.y -> " << y << ", pose.z -> "<< z << endl;
		Eigen::Vector3f landmark_position(x,y,z);
		//give landmark coordinates in robot frame, and project it to cartesian plan (ignore z axis)
		Eigen::Vector3f camera_t(0., 0., 0.);
		Eigen::Matrix3f camera_R;
		camera_R << 
			0, 0, 1,							//landmark z axis = robot x axis
			1, 0, 0,						    //landmark x axis = robot y axis
			0, -1, 0;						    //landmark y axis opposite at robot z axis
		
		Eigen::Vector3f lm_wrt_robot = camera_R * landmark_position + camera_t;
		Eigen::Vector2f landmark_pos(lm_wrt_robot.x(), lm_wrt_robot.y());
		tag_transforms[id][amount_detections[id]++] = landmark_pos;
	}
}



void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
	/* debug print
	cerr << "[ODOM:" << odom_msg->header.seq << "] Position of mobile base at time: " << odom_msg->header.stamp << endl;
	cerr << "Position:\n\tX: " << odom_msg->pose.pose.position.x << endl;
	cerr << "\tY: " << odom_msg->pose.pose.position.y << endl;
	cerr << "\tZ: " << odom_msg->pose.pose.position.z << endl;
	
	cerr << "Orientation:\n\tX: " << odom_msg->pose.pose.orientation.x << endl;
	cerr << "\tY: " << odom_msg->pose.pose.orientation.y << endl;
	cerr << "\tZ: " << odom_msg->pose.pose.orientation.z << endl;
	cerr << "\tW: " << odom_msg->pose.pose.orientation.w << endl;
	*/ 
	//get robot informations (in 2D)
	if(stop) return;
	curr_robot_pose.x = odom_msg->pose.pose.position.x;
	curr_robot_pose.y = odom_msg->pose.pose.position.y;
	tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	curr_robot_pose.theta = yaw;
}


int main(int argc, char** argv) {
	
	ros::init(argc, argv, "apriltag_image_listener_node");
	ros::NodeHandle nh("~");
	
	string scan_topic, apriltag_img_topic, bag_img_topic, tag_detect_topic, camera_info_topic, apriltag_cam_info_topic, odom_topic;
	//nh.param("scan_topic", scan_topic, string("/scan"));
	nh.param("apriltag_img_topic", apriltag_img_topic, string("/camera/image_rect"));
	nh.param("bag_img_topic", bag_img_topic, string("/usb_cam/image_raw"));
	nh.param("tag_detect_topic", tag_detect_topic, string("/tag_detections"));
	nh.param("camera_info_topic", camera_info_topic, string("/camera/camera_info"));
	nh.param("odom_topic", odom_topic, string("/odom"));
	
	//subscribing to topics of the bag...
	//ros::Subscriber sub_scan = nh.subscribe(scan_topic, 100, scanCallback);
	//cout << "Subcribed to topic " << scan_topic << endl;
	ros::Subscriber sub_usb_cam = nh.subscribe(bag_img_topic, 100, captureImageCallback);
	//cerr << "Subcribed to topic " << bag_img_topic << endl;
	ros::Subscriber sub_odom = nh.subscribe(odom_topic, 100, odomCallback);
	//cerr << "Subcribed to topic " << odom_topic << endl;
	
	//subscribe to topic /tag_detections of apriltag node
	ros::Subscriber sub_detect = nh.subscribe(tag_detect_topic, 100, detectionCallback);
	//cerr << "Subcribed to topic " << tag_detect_topic << endl;
	
	//publish messages to apriltag_ros node
	img_pub = nh.advertise<sensor_msgs::Image>(apriltag_img_topic,100);
	//cerr << "Publishing on topic " << apriltag_img_topic << endl;
	camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 100);
	//cerr << "Publishing on topic " << camera_info_topic << endl;
	
	for(int c=0; c < 14; c++) {
		is_detected[c] = false;
		amount_detections[c] = 0;
	}
	std::ofstream graphFile;
	cout << "Opening file thesis.g2o... ";
	graphFile.open("thesis.g2o", ios::out | ios::trunc);
	cout << "Done." << endl;
	
	//initial pose
	prev_robot_pose.x = 0;
	prev_robot_pose.y = 0;
	prev_robot_pose.theta = 0;
	graphFile << label_VERTEX_SE2 << " " << robot_pose_node_id-1 << " " << prev_robot_pose.x << " " << prev_robot_pose.y << " " << prev_robot_pose.theta << endl;
	cout << "Stored initial robot pose node with ID " << robot_pose_node_id-1 << endl;
	
	ros::Duration d(5);										//print out every 5 seconds
	cout << "Waiting for acquiring data..." << endl;
	while(ros::ok()) {
		stop = true;
		
		//build geometrical objects from data
		Eigen::Matrix2f prev_robot_R, curr_robot_R;
		prev_robot_R << cos(prev_robot_pose.theta), -sin(prev_robot_pose.theta), sin(prev_robot_pose.theta), cos(prev_robot_pose.theta);
		curr_robot_R << cos(curr_robot_pose.theta), -sin(curr_robot_pose.theta), sin(curr_robot_pose.theta), cos(curr_robot_pose.theta);
		Eigen::Vector2f prev_robot_t = Eigen::Vector2f(prev_robot_pose.x, prev_robot_pose.y);
		Eigen::Vector2f curr_robot_t = Eigen::Vector2f(curr_robot_pose.x, curr_robot_pose.y); 
		
		//write odom nodes
		if(curr_robot_pose.x != prev_robot_pose.x || curr_robot_pose.y != prev_robot_pose.y) {
			if(robot_pose_node_id <= ROBOT_POSE_ID_LIMIT) {
				graphFile << label_VERTEX_SE2 << " " << robot_pose_node_id << " " << curr_robot_pose.x << " " << curr_robot_pose.y << " " << curr_robot_pose.theta << endl;
				cout << "Stored robot pose node with ID " << robot_pose_node_id << endl;
				
				//write pose-to-pose edges
				//Eigen::Matrix2f transposed_prev_R = prev_robot_R.transpose();
				float da = prev_robot_pose.theta - curr_robot_pose.theta;							//angle of edge
				Eigen::Matrix2f R;
				R << cos(da), -sin(da), sin(da), cos(da);
				Eigen::Vector2f dt = R * (curr_robot_t - prev_robot_t); 							//distance between current and previous poses (wrt previous pose)															
				graphFile << label_EDGE_SE2 << " " << robot_pose_node_id-1 << " " << robot_pose_node_id << " " << dt.x() << " " << dt.y() << " " << da << " " 
					<< 500 << " " << 0 << " " << 0 << " " << 500 << " " << 0 << " " << 5000 << endl;						//other stuff
				cout << "Stored pose-to-pose edge " << robot_pose_node_id-1 << " -> " << robot_pose_node_id << endl;
				
				//update
				robot_pose_node_id++;
				prev_robot_pose = curr_robot_pose;
			} 
			else ros::shutdown();
		}
		
		//write tag nodes
		int i,n,m;
		for(i=0; i < 14; ++i) {
			n = amount_detections[i];
			if(n > 1) {
				//average of all detections
				float landmark_x=0, landmark_y=0;
				for(m = 0; m < n; ++m) {
					landmark_x += tag_transforms[i][m].x();
					landmark_y += tag_transforms[i][m].y(); 
				}
				float avg_landmark_x = landmark_x / n;
				float avg_landmark_y = landmark_y / n;
				Eigen::Vector2f avg_landmark = Eigen::Vector2f(avg_landmark_x, avg_landmark_y);
				if(!is_detected[i]) {																//first time we saw the landmark i
					Eigen::Vector2f avg_landmark_pos = curr_robot_R * avg_landmark + curr_robot_t;
					graphFile << label_VERTEX_XY << " " << i << " " << avg_landmark_pos.x() << " " << avg_landmark_pos.y() << endl;
					cout << "Stored tag pose node with ID " << i << endl;
					is_detected[i] = true;
				} 
				else cout << "Tag " << i << " already detected." << endl;
				
				//write pose-to-tag edges
				graphFile << label_EDGE_SE2_XY << " " << robot_pose_node_id << " " << i << " " << avg_landmark.x() << " " << avg_landmark.y() << " " 
					<< 1000 << " " << 0 << " " << 1000 << endl; 		//other stuff				
				cout << "Stored pose-to-landmark edge " << robot_pose_node_id-1 << " -> " << i << endl;
			} 
			amount_detections[i] = 0;
		}
		
		stop = false;
		ros::spinOnce();
		d.sleep();
	}
	
	graphFile.close();
	cout << "Graph completed." << endl; 
	return 0;
}
