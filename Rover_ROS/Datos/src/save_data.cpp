#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <fstream>
#include <ros/ros.h>

using namespace std;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;

double mag_yaw = 0.0;
double yaw_odom = 0.0;
double x_gps = 0.0;
double y_gps = 0.0;
double x_odom = 0.0;
double y_odom = 0.0;
double enc_right = 0.0;
double enc_left = 0.0;

void orientation(const OdometryPtr& yaw){
	mag_yaw = yaw->pose.pose.position.y;
}

void gps_info(const Vector3StampedPtr& data){
	x_gps = data->vector.x;
	y_gps = data->vector.y;
}

void encoder(const OdometryPtr& odom){
	x_odom = odom->pose.pose.position.x;
	y_odom = odom->pose.pose.position.y;
	yaw_odom = odom->pose.pose.position.z;
}

void read_enc(const JointStatePtr& enc){
	enc_right = enc->velocity[4];
	enc_left = enc->velocity[5];
}

void principal(){
	ros::Rate rate(10);
	while(ros::ok()){
		ofstream f("datos.txt", ios::app);
		f << fixed << setprecision(10) << mag_yaw << "\t" << yaw_odom << "\t" << x_gps << "\t" << y_gps << "\t" << x_odom << "\t" << y_odom << "\t" << enc_right << "\t" << enc_left << "\n";

		std::cout << setprecision(10) << mag_yaw << "\t" << yaw_odom << "\t" << x_gps << "\t" << y_gps << "\t" << x_odom << "\t" << y_odom << "\n";
		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "save_data");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/yaw", 1000, orientation);
	ros::Subscriber sub1 = nh.subscribe("/gps", 1000, gps_info);
	ros::Subscriber sub2 = nh.subscribe("/position", 1000, encoder);
	ros::Subscriber sub3 = nh.subscribe("/enc_data", 1000, read_enc);

	principal();
	return 0;
}
