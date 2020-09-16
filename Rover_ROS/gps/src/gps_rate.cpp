#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;
using namespace geometry_msgs;

double x = 0.0;
double y = 0.0;

ros::Publisher pub;

void gps(const Vector3StampedPtr& data){
	x = data->vector.x;
	y = data->vector.y;
}

void principal(){
	ros::Rate rate(10);
	while(ros::ok()){
		geometry_msgs::Vector3Stamped msg;
		msg.vector.x = x;
		msg.vector.y = y;
		pub.publish(msg);

		//std::cout << x << "\t" << y << "\n";

		ros::spinOnce();
		rate.sleep();
	}
}


int main(int argc, char **argv){
	ros::init(argc, argv, "gps_rate");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/data_gps", 1000, gps);
	pub = nh.advertise<geometry_msgs::Vector3Stamped>("/gps", 1000);

	std::cout << "nodo GPS_RATE creado" << "\n";

	principal();
	return 0;
}
