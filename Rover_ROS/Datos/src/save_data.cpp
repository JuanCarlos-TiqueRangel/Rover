#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
#include <ros/ros.h>

using namespace std;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;

using namespace message_filters;

class Data{

        public:
                float pos1;
                float pos2;
                float pos3;
		float pos4;
		float pos5;
		float pos6;
		float pos7;
		float pos8;
		float pos9;
		float pos10;
		float pos11;

                void callback(const JointStateConstPtr& dutty, const OdometryConstPtr& gps_data, const OdometryConstPtr& enc_vel,
			const ImuConstPtr& acc, const Vector3StampedConstPtr& mag_data);

        };

void Data::callback(const JointStateConstPtr& dutty, const OdometryConstPtr& gps_data, const OdometryConstPtr& enc_vel,
	const ImuConstPtr& acc, const Vector3StampedConstPtr& mag_data){

        pos1 = dutty->velocity[0];
        pos2 = dutty->velocity[1];
	pos3 = gps_data->pose.pose.position.x;
	pos4 = enc_vel->twist.covariance[5];
	pos5 = enc_vel->twist.covariance[6];
	pos6 = acc->linear_acceleration.x;
	pos7 = acc->linear_acceleration.y;
	pos8 = acc->linear_acceleration.z;
	pos9 = mag_data->vector.x;
	pos10 = mag_data->vector.y;
	pos11 = mag_data->vector.z;

        ros::Rate ros_time(10);

        ofstream f("datos.txt", ios::app);
        f << pos1 <<" "<< pos2 <<" "<< pos3 << '\n';
        f.close();

        std::cout << pos1 << "\t" << pos2 << "\t" << pos3  << "\t" << pos4 << "\t" << pos4 << "\t" << pos6 << "\t" << pos7  << "\t" << pos8 << "\t" << pos9 << "\t" << pos10 << "\n";
        ros_time.sleep();
}

int main (int argc, char **argv){

        ros::init(argc, argv, "odome");
        ros::NodeHandle nh;

	message_filters::Subscriber<JointState> pwm(nh, "/PWM", 1);
	//message_filters::Subscriber<Odometry> enc(nh, "/enc_data", 1);
	message_filters::Subscriber<Odometry> vel(nh, "/Position", 1);
	message_filters::Subscriber<Odometry> gps(nh, "/gps_dis", 1);
	message_filters::Subscriber<Imu> imu_acc(nh, "/mti/sensor/imu_free", 1);
	message_filters::Subscriber<Vector3Stamped> imu_mag(nh, "/mti/sensor/magnetic", 1);
	//message_filters::Subscriber<MagneticField> mag(nh, "/magnetic", 1);

        Data data;

        //typedef sync_policies::ExactTime<JointState, Odometry, Odometry, Odometry, MagneticField, Imu, Vector3Stamped> MySyncPolicy;
        typedef sync_policies::ApproximateTime<JointState, Odometry, Odometry, Imu, Vector3Stamped> MySyncPolicy;
        Synchronizer<MySyncPolicy> sync(MySyncPolicy(10000), pwm, gps, vel, imu_acc, imu_mag);


        //TimeSynchronizer<JointState, Odometry, Odometry, Imu, Vector3Stamped> sync(pwm,  gps, vel, imu_acc, imu_mag, 1);
        sync.registerCallback(boost::bind(&Data::callback, &data, _1, _2, _3, _4, _5));
        ros::spin();
        return 0;
}
