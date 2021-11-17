#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class TransformEstimator {

private:
	ros::NodeHandle n;
	
	message_filters::Subscriber<nav_msgs::Odometry> odom; 
	message_filters::Subscriber<geometry_msgs::PoseStamped> gt; 

	ros::Publisher quaternion;

	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseStamped> MySyncPolicy;
	
	message_filters::Synchronizer<MySyncPolicy> sync; 
public:


	TransformEstimator():sync(MySyncPolicy(10), odom, gt){
		odom.subscribe(n, "scout_odom", 1);
		gt.subscribe(n, "gt_pose", 1);

		quaternion = n.advertise<geometry_msgs::Quaternion>("transformation_quaternion", 1000);
		sync.registerCallback(boost::bind(&TransformEstimator::ComputeTransform, this, _1, _2));
	} 

	void ComputeTransform(const nav_msgs::Odometry::ConstPtr& odom, const geometry_msgs::PoseStamped::ConstPtr& gt){

	double theta_odom = tf::getYaw(odom->pose.pose.orientation);
	double theta_gt = tf::getYaw(gt->pose.orientation);

	double theta_diff = theta_gt-theta_odom;

	double a = (gt->pose.position.x)-(odom->pose.pose.position.x)*(std::cos(theta_diff))+(odom->pose.pose.position.y)*(std::sin(theta_diff));
	double b = (gt->pose.position.y)-(odom->pose.pose.position.x)*(std::sin(theta_diff))-(odom->pose.pose.position.y)*(std::cos(theta_diff));
	tf::Quaternion transform_q;
    transform_q.setRPY(0, 0, theta_diff);


    ROS_INFO("x = %f",a);
    ROS_INFO("y = %f",b);
    ROS_INFO("z = %f",(gt->pose.position.z));

    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = 0;
    quat_msg.y = 0;
    quat_msg.z = transform_q[2];
    quat_msg.w = transform_q[3];

    quaternion.publish(quat_msg);
	}
};



int main (int argc, char** argv){
	ros::init(argc, argv, "Transform");

	
	TransformEstimator transform;
	ros::spin();

	

	return 0;
}