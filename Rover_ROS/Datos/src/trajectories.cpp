#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <fstream>
#include <ros/ros.h>

using namespace std;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;

double yaw_kalman = 0.0;
double yaw_odom = 0.0;
double x_gps = 0.0;
double y_gps = 0.0;
double x_kalman = 0.0;
double y_kalman = 0.0;
double imu_acc_x = 0.0;
double imu_acc_y = 0.0;
double imu_acc_z = 0.0;
double imu_vel_x = 0.0;
double imu_vel_y = 0.0;
double imu_vel_z = 0.0;
double enc_right = 0.0;
double enc_left = 0.0;
double odom_x = 0.0;
double odom_y = 0.0;

double sL = 0.0;
double sR = 0.0;
double sL_F = 0.0;
double sR_F = 0.0;

double imu_acc_Rx = 0.0;
double imu_acc_Ry = 0.0;
double imu_yaw = 0.0;
double mag_yaw = 0.0;

void kalman(const Vector3StampedPtr& data){
	x_kalman = data->vector.x;
	y_kalman = data->vector.y;
        yaw_kalman = data->vector.z;
}

void gps_info(const Vector3StampedPtr& data){
        x_gps = data->vector.x;
        y_gps = data->vector.y;
}

void imu_info(const ImuPtr& imu){
	imu_acc_x = imu->linear_acceleration.x;
	imu_acc_y = imu->linear_acceleration.y;
	imu_acc_z = imu->linear_acceleration.z;

	imu_vel_x = imu->angular_velocity.x;
	imu_vel_y = imu->angular_velocity.y;
	imu_vel_z = imu->angular_velocity.z;
}

void read_enc(const JointStatePtr& enc){
        enc_right = enc->velocity[3];
        enc_left = enc->velocity[4];
}

void odometria(const OdometryPtr& odom){
	odom_x = odom->pose.pose.position.x;
	odom_y = odom->pose.pose.position.y;

	sL_F = odom->pose.pose.orientation.x;
	sR_F = odom->pose.pose.orientation.y;
	sL = odom->pose.pose.orientation.z;
	sR = odom->pose.pose.orientation.w;

}


void imu_rotate(const ImuPtr& imu){
	imu_yaw = imu->orientation.x;
	mag_yaw = imu->orientation.y;

	imu_acc_Rx = imu->linear_acceleration.x;
	imu_acc_Ry = imu->linear_acceleration.y;
}


void principal(){
        ros::Rate rate(10);
        while(ros::ok()){
                ofstream f("datos.txt", ios::app);
                f << fixed << setprecision(10)
		<< imu_yaw << 	 "\t" << mag_yaw << 	"\t" << imu_acc_Rx <<	"\t"
		<< imu_acc_Ry << "\t" << yaw_kalman << 	"\t" << x_gps <<	"\t"
		<< y_gps << 	 "\t" << x_kalman << 	"\t" << y_kalman <<	"\t"
		<< imu_acc_x <<	 "\t" << imu_acc_y << 	"\t" << imu_acc_z <<	"\t"
		<< imu_vel_x <<  "\t" << imu_vel_y << 	"\t" << imu_vel_z <<	"\t"
		<< enc_left << 	 "\t" << enc_right <<  	"\t" << odom_x << 	"\t"
		<< odom_y << 	 "\t" << sL_F << 	"\t" << sR_F << 	"\t"
		<< sL << 	 "\t" << sR << 		"\n";

                std::cout << setprecision(10)
                << imu_yaw <<    "\t" << mag_yaw <<     "\t" << imu_acc_Rx <<   "\t"
                << imu_acc_Ry << "\t" << yaw_kalman <<  "\t" << x_gps <<        "\t"
                << y_gps <<      "\t" << x_kalman <<    "\t" << y_kalman <<     "\t"
                << imu_acc_x <<  "\t" << imu_acc_y <<   "\t" << imu_acc_z <<    "\t"
                << imu_vel_x <<  "\t" << imu_vel_y <<   "\t" << imu_vel_z <<    "\t"
                << enc_left <<   "\t" << enc_right <<   "\t" << odom_x <<       "\t"
                << odom_y <<     "\t" << sL_F <<        "\t" << sR_F <<         "\t"
                << sL <<         "\t" << sR <<          "\n";
                ros::spinOnce();
                rate.sleep();

	/* imu_yaw---->	[1]
	mag_yaw	------>	[2]
	imu_acc_Rx---->	[3]
	imu_acc_Ry---->	[4]
	yaw_kalman---->	[5]
	x_gps--------->	[6]
	y_gps--------->	[7]
	x_kalman------>	[8]
	y_kalman------>	[9]
	imu_acc_x----->	[10]
	imu_acc_y----->	[11]
	imu_acc_z----->	[12]
	imu_vel_x-----> [13]
	imu_vel_y----->	[14]
	imu_vel_z----->	[15]
	enc_left------>	[16]
	enc_right----->	[17]
	odom_x-------->	[18]
	odom_y-------->	[19]
	SL_F---------->	[20]
	SR_F---------->	[21]
	SL------------>	[22]
	SR------------>	[23]
	 */

        }
}

int main(int argc, char **argv){
        ros::init(argc, argv, "save_data");
        ros::NodeHandle nh;

        ros::Subscriber sub = nh.subscribe("/kalman_filter", 1000, kalman);
        ros::Subscriber sub1 = nh.subscribe("/gps", 1000, gps_info);
	ros::Subscriber sub2 = nh.subscribe("/imu/data", 1000, imu_info);
	ros::Subscriber sub3 = nh.subscribe("/enc_data", 1000, read_enc);
	ros::Subscriber sub4 = nh.subscribe("/odom", 1000, odometria);
	ros::Subscriber sub5 = nh.subscribe("/yaw", 1000, imu_rotate);

	std::cout << "GUARDANDO TRAJECTORIAS" << "\n";

        principal();
        return 0;
}
