#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include "project/CustomOdometry.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include "project/SetPose.h"
#include "project/ResetPose.h"
#include <dynamic_reconfigure/server.h>
#include <project/parametersConfig.h>
#include <sstream>

class DifferentialDrive{
private:


	const double GR = 38.51601667;
	const double APP_BASELINE = 1.011065778;
	const double CIRCUMFERENCE = 2*M_PI*0.1575;

	ros::NodeHandle n;

	double x; 
	double y;
    double theta;

    double v_linear = 0;
    double omega = 0;

    bool integration = true;

    ros::Time current_time = ros::Time(0,0);

	


	ros::Publisher velocity;
	ros::Publisher computed_odometry;
	ros::Publisher computed_odometry_custom;
	tf::TransformBroadcaster transform_broadcaster;
	


	message_filters::Subscriber<robotics_hw1::MotorSpeed> frontleft;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> rearleft;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> frontright;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> rearright;


	message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> 
	sync;

	ros::ServiceServer service_set; 
	ros::ServiceServer service_reset; 

public:

	DifferentialDrive():sync(frontleft, rearleft, frontright, rearright, 10){

		frontleft.subscribe(n, "motor_speed_fl", 1);
		frontright.subscribe(n, "motor_speed_fr", 1);
		rearleft.subscribe(n, "motor_speed_rl", 1);
		rearright.subscribe(n, "motor_speed_rr", 1);


		n.getParam("/x", x);
	 	n.getParam("/y", y);
		n.getParam("/theta", theta);


		service_set = n.advertiseService("set_pose", &DifferentialDrive::SetPose, this);
		service_reset = n.advertiseService("reset_pose", &DifferentialDrive::ResetPose, this);


		sync.registerCallback(boost::bind(&DifferentialDrive::ComputeOdometry, this, _1, _2, _3, _4));


		velocity = n.advertise<geometry_msgs::TwistStamped>("my_velocity", 1000);
		computed_odometry = n.advertise<nav_msgs::Odometry>("my_odometry", 1000);
		computed_odometry_custom = n.advertise<project::CustomOdometry>("my_custom_odometry", 1000);
	}



	void ComputeOdometry(
			  const robotics_hw1::MotorSpeed::ConstPtr& frontleft,
		      const robotics_hw1::MotorSpeed::ConstPtr& rearleft,
		      const robotics_hw1::MotorSpeed::ConstPtr& frontright,
		      const robotics_hw1::MotorSpeed::ConstPtr& rearright)
	{


		const ros::Time& next_time = frontleft->header.stamp;
	    double delta_t = (next_time - current_time).toSec();
	    current_time = next_time;

	    double v_l = -(frontleft->rpm + rearleft->rpm)*CIRCUMFERENCE/(120*GR);
	    double v_r =  (frontright->rpm + rearright->rpm)*CIRCUMFERENCE/(120*GR);

	    v_linear = ((v_r + v_l)/2);
	    omega = ((v_r - v_l)/APP_BASELINE);


	    //EULER
	    if(integration){
	    	x = x + v_linear * delta_t * std::cos(theta);
	    	y = y + v_linear * delta_t * std::sin(theta);
	    	theta = theta + omega * delta_t;
		}

		//RUNGE_KUTTA
		else{
			x = x + v_linear * delta_t * std::cos(theta + (omega*delta_t/2));
	    	y = y + v_linear * delta_t * std::sin(theta + (omega*delta_t/2));
	    	theta = theta + omega * delta_t;
		}

		PublishVelocity(v_linear, omega);
		PublishOdometry();
		PublishOdometryTF();
		PublishOdometryCustom(integration);
	}


	//Method called by the dynamic_reconfigure callback function to set the integration method
	void setIntegration(bool activate){
		this->integration = activate;
	}


	//Method used by set_pose service, takes the arguements passed and sets the current position and orientation
	bool SetPose(project::SetPose::Request &req, project::SetPose::Response &res){
		this->x = req.x;
		this->y = req.y;
		this->theta = req.theta;
		return true;
	}


	//Method used by reset_pose service, sets x=0, y=0, theta=0
	bool ResetPose(project::ResetPose::Request &req, project::ResetPose::Response &res){
		this->x = 0.0;
		this->y = 0.0;
		this->theta = 0.0;
		return true;
	}


	//Method used to publish angular and linear velocity as geometry_msgs/TwistStamped in /my_velocity
	void PublishVelocity(float v_linear, float omega){
		geometry_msgs::TwistStamped velocity_msg;
		
		velocity_msg.header.stamp = ros::Time::now();
		velocity_msg.header.frame_id = "world";

		velocity_msg.twist.linear.x = v_linear;
		velocity_msg.twist.linear.y = 0.0;
		velocity_msg.twist.linear.z = 0.0;

		velocity_msg.twist.angular.x = 0.0;
		velocity_msg.twist.angular.y= 0.0;
		velocity_msg.twist.angular.z = omega;

		velocity.publish(velocity_msg);
	}


	//Method used to publish the computed odometry as nav_msgs/Odometry in /my_odometry
	void PublishOdometry(){
		nav_msgs::Odometry odometry_msg;

		odometry_msg.header.stamp = ros::Time::now();
		odometry_msg.header.frame_id = "odom";

		odometry_msg.child_frame_id = "base_link";
		
		odometry_msg.pose.pose.position.x = x;
		odometry_msg.pose.pose.position.y = y;
		odometry_msg.pose.pose.position.z = 0.0;
		odometry_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

		odometry_msg.twist.twist.linear.x = v_linear;
		odometry_msg.twist.twist.angular.z = omega;

		computed_odometry.publish(odometry_msg);
	}


	//Method  used to broadcast the tf between odom and base_link
	void PublishOdometryTF(){
		tf::Transform transform;

		transform.setOrigin(tf::Vector3(this->x, this->y, 0));
		transform.setRotation(tf::createQuaternionFromRPY(0, 0, this->theta));

		transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
	}


	//Method used to publish both odometry and integration type as project/CustomOdometry in /my_custom_odometry
	void PublishOdometryCustom(bool integration_method){
		project::CustomOdometry custom_msg;

		nav_msgs::Odometry odometry_msg;

		odometry_msg.header.stamp = ros::Time::now();
		odometry_msg.header.frame_id = "odom";

		odometry_msg.child_frame_id = "base_link";
		
		odometry_msg.pose.pose.position.x = x;
		odometry_msg.pose.pose.position.y = y;
		odometry_msg.pose.pose.position.z = 0.0;
		odometry_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

		odometry_msg.twist.twist.linear.x = v_linear;
		odometry_msg.twist.twist.angular.z = omega;

		custom_msg.odom = odometry_msg;
		if(integration_method) custom_msg.method = "euler";
		else custom_msg.method = "rk";

		computed_odometry_custom.publish(custom_msg);

	}
};



void decideIntegrationMethod(project::parametersConfig &config, uint32_t level, DifferentialDrive &odometry ){
	if(config.integration_type == 0) odometry.setIntegration(true);
	else odometry.setIntegration(false);
}


int main (int argc, char** argv){
	ros::init(argc, argv, "DifferentialDrive");

	DifferentialDrive differtialDriveOdometry;

	dynamic_reconfigure::Server<project::parametersConfig> server;

	server.setCallback(boost::bind(&decideIntegrationMethod, _1, _2, std::ref(differtialDriveOdometry)));

	ros::spin();
	return 0;
}
