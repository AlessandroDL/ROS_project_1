#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>

class estimator {
//GR estimation
int gr_times = 0;
double gr = 35;
double gr_left, gr_right,gr_linear, actual_linear, estimated_gr; 

//Apparent Baseline estimation
int baseline_times = 0;
double baseline = 0.583; 
double omega, actual_omega, estimated_baseline;
double theta;

private:
	ros:: NodeHandle n;


	message_filters::Subscriber<robotics_hw1::MotorSpeed> frontleft;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> rearleft;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> frontright;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> rearright;
	message_filters::Subscriber<nav_msgs::Odometry> odom;

	message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, nav_msgs::Odometry> 
	sync;

public: 
	estimator():sync(frontleft, rearleft, frontright, rearright, odom, 10){
		frontleft.subscribe(n, "motor_speed_fl", 1);
		frontright.subscribe(n, "motor_speed_fr", 1);
		rearleft.subscribe(n, "motor_speed_rl", 1);
		rearright.subscribe(n, "motor_speed_rr", 1);
		odom.subscribe(n, "scout_odom", 1);


		sync.registerCallback(boost::bind(&estimator::callback, this, _1, _2, _3, _4, _5));

	}

	void callback (const robotics_hw1::MotorSpeed::ConstPtr& msg1,
		      const robotics_hw1::MotorSpeed::ConstPtr& msg2,
		      const robotics_hw1::MotorSpeed::ConstPtr& msg3,
		      const robotics_hw1::MotorSpeed::ConstPtr& msg4,
		      const nav_msgs::Odometry::ConstPtr& msg5){

		actual_linear = msg5->twist.twist.linear.x;
		actual_omega = msg5->twist.twist.angular.z;
		
		gr_left = -((msg1->rpm)+(msg2->rpm))*(2*M_PI*0.1575)/120;
		gr_right = ((msg3->rpm)+(msg4->rpm))*(2*M_PI*0.1575)/120;
		gr_linear = (gr_left+gr_right)/2;


		if(actual_linear != 0){
			gr_times++;
			gr=gr+gr_linear/actual_linear;
			estimated_gr=gr/gr_times;
		}
		if(actual_omega!= 0){
			baseline_times++;
			omega = (-gr_left/estimated_gr)+(gr_right/estimated_gr);
			baseline = baseline + omega/actual_omega;
			estimated_baseline = baseline/baseline_times;
		}

		
		ROS_INFO("calculated linear velocity is (%f)", gr_linear);
		ROS_INFO("actual linear velocity is (%f)", actual_linear);
		ROS_INFO("GR = (%f)", estimated_gr);
		ROS_INFO("2Yo = (%f)\n", estimated_baseline);

	}


};

int main (int argc, char** argv){
	ros::init(argc, argv, "estimator");

	estimator gr_and_baseline_estimator;

	ros::spin();
	return 0;
}