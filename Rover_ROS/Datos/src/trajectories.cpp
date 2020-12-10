#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gps_common/GPSFix.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/MagneticField.h>

#include <fstream>
#include <stdio.h>

using namespace std;
using namespace control_msgs;
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

// VARIABLES CONTROL TRAYECTORIA
double PT_x, PT_y, wpx_1, wpy_1, wpx_2, wpy_2;
double angle_goal;
double Vd, Vd_pwm;
double du, du_pwm;
double error_yaw;
double distancia_PT, distancia_wp;

// VARIABLES MAGNETOMETRO
double mag_gps = 0.0;

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

        enc_right = odom->twist.twist.linear.y;
        enc_left = odom->twist.twist.linear.x;
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

void control_data(const JointTrajectoryControllerStatePtr& mpc){
	PT_x = mpc->desired.positions[0];
	PT_y = mpc->desired.positions[1];
	wpx_1 = mpc->desired.positions[2];
	wpy_1 = mpc->desired.positions[3];
	wpx_2 = mpc->desired.positions[4];
	wpy_2 = mpc->desired.positions[5];
	angle_goal = mpc->desired.effort[0];
	Vd = mpc->actual.velocities[1];
	Vd_pwm = mpc->actual.velocities[0];
	du = mpc->actual.effort[1];
	du_pwm = mpc->actual.effort[0];
	error_yaw = mpc->error.effort[0];
	distancia_PT = mpc->error.positions[0];
	distancia_wp = mpc->error.positions[1];
}

void compass_gps(const MagneticFieldPtr& mag){
	mag_gps = mag->magnetic_field.z;
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
		<< mag_yaw <<	 "\t" << imu_vel_fz <<  "\t" << imu_acc_Rx << 	"\t"
		<< imu_acc_Ry << "\t" << imu_acc_x << 	"\t" << imu_acc_y <<	"\t"
		<< imu_acc_z <<	 "\t" << imu_vel_z <<	"\t" << PT_x <<	 	"\t"
		<< PT_y <<	 "\t" << wpx_1 <<	"\t" << wpy_1 <<	"\t"
		<< wpx_2 <<	 "\t" << wpy_2 <<	"\t" << angle_goal << 	"\t"
		<< Vd << 	 "\t" << Vd_pwm << 	"\t" << du << 		"\t"
		<< du_pwm << 	 "\t" << error_yaw << 	"\t" << distancia_PT << "\t"
		<< distancia_wp << "\t"	<< mag_gps <<	"\n";

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
		lspeed_right	[10]
		lspeed_left	[11]
		imu_yaw		[12]
		mag_yaw		[13]
		imu_vel_z	[14]
		imu_acc_Rx	[15]
		imu_acc_Ry	[16]
		imu_acc_x	[17]
		imu_acc_y	[18]
		imu_acc_z	[19]
		imu_vel_z	[20]
		PT_x 		[21]
		PT_y 		[22]
		wpx_1		[23]
		wpy_1 		[24]
		wpx_2		[25]
		wpy_2 		[26]
		angle_goal 	[27]
		Vd 		[28]
		Vd_pwm 		[29]
		du 		[30]
		du_pwm 		[31]
		error_yaw 	[32]
		distancia_PT 	[33]
		distancia_wp 	[34]
		mag_gps		[35]

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
	ros::Subscriber sub7 = nh.subscribe("/MPC", 1000, control_data);
	ros::Subscriber sub8 = nh.subscribe("/mag_calibrated", 1000, compass_gps);

        std::cout << "GUARDANDO TRAJECTORIAS" << "\n";

	principal();
	return 0;
}
