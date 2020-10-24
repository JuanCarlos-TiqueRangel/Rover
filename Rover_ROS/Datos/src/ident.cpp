#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <fstream>

using namespace std;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;

double imu_yaw = 0.0;
double mag_yaw = 0.0;

double st = 0.0;
double th = 0.0;

void imu_rotate(const ImuPtr& imu){
        imu_yaw = imu->orientation.x;
        mag_yaw = imu->orientation.y;
}

void pwm_data(const JointStatePtr& pwm){
	th = pwm->velocity[0];
	st = pwm->velocity[1];
}

void principal(){
	ros::Rate rate(20);
	while(ros::ok()){

                ofstream f("datos.txt", ios::app);
                f << fixed << setprecision(10)
		<< imu_yaw << 	 "\t" << mag_yaw << 	"\t" << th <<	"\t"
		<< st << 	 "\n";

		std::cout << setprecision(10)
		<< imu_yaw <<    "\t" << mag_yaw <<     "\t" << th <<   "\t"
		<< st <<         "\n";

		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc, char **argv){
        ros::init(argc, argv, "save_data");
        ros::NodeHandle nh;

        ros::Subscriber sub5 = nh.subscribe("/yaw", 1000, imu_rotate);
	ros::Subscriber sub6 = nh.subscribe("/main_node", 1000, pwm_data);

        std::cout << "GUARDANDO TRAJECTORIAS" << "\n";

	principal();
        return 0;
}
