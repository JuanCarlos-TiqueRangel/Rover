#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;

double x = 0.0;
double y = 0.0;
double z = 0.0;
int satelites = 0;
double speed = 0.0;

ros::Publisher pub;

void gps(const OdometryPtr& data){
	x = data->pose.pose.position.x;
	y = data->pose.pose.position.y;
	z = data->pose.pose.position.z;
	speed = data->twist.twist.linear.x;
}

void principal(){
	ros::Rate rate(20);
	while(ros::ok()){
		nav_msgs::Odometry msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = " GPS_RATE_UTM ";
		msg.pose.pose.position.x = x;
		msg.pose.pose.position.y = y;
		msg.pose.pose.position.z = z;
		msg.twist.twist.linear.x = speed;
		pub.publish(msg);

		//std::cout << x << "\t" << y << "\n";

		ros::spinOnce();
		rate.sleep();
	}
}


int main(int argc, char **argv){
	ros::init(argc, argv, "gps_rate");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/gps/utm", 1000, gps);
	pub = nh.advertise<nav_msgs::Odometry>("/gps", 1000);

	std::cout << "nodo GPS_RATE creado" << "\n";

	principal();
	return 0;
}
