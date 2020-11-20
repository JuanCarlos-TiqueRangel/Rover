#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gps_common/GPSFix.h>

#include <fstream>
#include <stdio.h>

using namespace std;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace gps_common;

// VARIABLES KALMAN
double yaw_kalman = 0.0;
double x_kalman = 0.0;
double y_kalman = 0.0;

// VARIABLES GPS
double x_gps = 0.0;
double y_gps = 0.0;
double gps_speed = 0.0;

// VARIABLES ENCODERS
double enc_right = 0.0;
double enc_left = 0.0;
double odom_x = 0.0;
double odom_y = 0.0;
double yaw_odom = 0.0;

// VARIABLES IMU_FILTRADA
double imu_acc_Rx = 0.0;
double imu_acc_Ry = 0.0;
double imu_vel_fz = 0.0;
double imu_yaw = 0.0;
double mag_yaw = 0.0;
int contador = 0;

// VARIABLE IMU
double imu_acc_x = 0.0;
double imu_acc_y = 0.0;
double imu_acc_z = 0.0;
double imu_vel_z = 0.0;


void kalman(const OdometryPtr& data){
        x_kalman = data->pose.pose.position.x;
        y_kalman = data->pose.pose.position.y;
        yaw_kalman = data->pose.pose.position.z;
}

void gps_info(const OdometryPtr& data){
        x_gps = data->pose.pose.position.x;
        y_gps = data->pose.pose.position.y;
}

void odometria(const OdometryPtr& odom){
        odom_x = odom->pose.pose.position.x;
        odom_y = odom->pose.pose.position.y;
	yaw_odom = odom->pose.pose.position.z;

        enc_right = odom->twist.twist.linear.x;
        enc_left = odom->twist.twist.linear.y;
}

void imu_rotate(const ImuPtr& imu){
        imu_yaw = imu->orientation.x;
        mag_yaw = imu->orientation.y;

	imu_vel_fz = imu->angular_velocity.z;
        imu_acc_Rx = imu->linear_acceleration.x;
        imu_acc_Ry = imu->linear_acceleration.y;
}


void gps_data(const GPSFixPtr& gps){
        gps_speed = gps->speed;
}

void imu_data(const ImuPtr& imu_info){
	imu_acc_x = imu_info->linear_acceleration.x;
	imu_acc_y = imu_info->linear_acceleration.y;
	imu_acc_z = imu_info->linear_acceleration.z;
	imu_vel_z = imu_info->angular_velocity.z;
}

void principal(){
	ros::Rate rate(20);
	while(ros::ok()){

                ofstream f("datos.txt", ios::app);
                f << fixed << setprecision(10)
		<< x_kalman << 	 "\t" << y_kalman << 	"\t" << yaw_kalman <<	"\t"
		<< x_gps <<	 "\t" << y_gps      << 	"\t" << gps_speed << 	"\t"
		<< odom_x <<	 "\t" << odom_y << 	"\t" << yaw_odom << 	"\t"
		<< enc_right <<	 "\t" << enc_left <<	"\t" << imu_yaw << 	"\t"
		<< mag_yaw <<	 "\t" << imu_vel_fz <<   "\t" << imu_acc_Rx << 	"\t"
		<< imu_acc_Ry << "\t" << imu_acc_x << 	"\t" << imu_acc_y <<	"\t"
		<< imu_acc_z <<	 "\t" << imu_vel_z <<	"\n";

                //std::cout << setprecision(10)
		//<< contador << "\n";
                /* << x_kalman <<   "\t" << y_kalman <<    "\t" << yaw_kalman <<   "\t"
                << x_gps <<      "\t" << y_gps      <<  "\t" << gps_speed <<    "\t"
                << odom_x <<     "\t" << odom_y <<      "\t" << yaw_odom <<     "\t"
                << enc_right <<  "\t" << enc_left <<    "\t" << imu_yaw <<      "\t"
                << mag_yaw <<    "\t" << imu_vel_z <<   "\t" << imu_acc_Rx <<   "\t"
                << imu_acc_Ry <<	"\n"; */

		ros::spinOnce();
		rate.sleep();
	}
}

/*
		x_kalman	[1]
		y_kalman	[2]
		yaw_kalman	[3]
		x_gps		[4]
		y_gps		[5]
		gps_speed	[6]
		odom_x		[7]
		odom_y		[8]
		yaw_odom	[9]
		speed_right	[10]
		speed_left	[11]
		imu_yaw		[12]
		mag_yaw		[13]
		imu_vel_z	[14]
		imu_acc_Rx	[15]
		imu_acc_Ry	[16]
		imu_acc_x	[17]
		imu_acc_y	[18]
		imu_acc_z	[19]
		imu_vel_z	[20]

*/


int main(int argc, char **argv){
        ros::init(argc, argv, "save_data");
        ros::NodeHandle nh;

        ros::Subscriber sub1 = nh.subscribe("/kalman_filter", 1000, kalman);
        ros::Subscriber sub2 = nh.subscribe("/gps", 1000, gps_info);
	ros::Subscriber sub3 = nh.subscribe("/gps/data", 1000, gps_data);
        ros::Subscriber sub4 = nh.subscribe("/odom", 1000, odometria);
        ros::Subscriber sub5 = nh.subscribe("/yaw", 1000, imu_rotate);
	ros::Subscriber sub6 = nh.subscribe("/imu/data", 1000, imu_data);

        std::cout << "GUARDANDO TRAJECTORIAS" << "\n";

	principal();
	return 0;
}
