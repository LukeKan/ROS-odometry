#include "ros/ros.h"
#include "robotics_project/floatStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


double WHEEL_BASELINE=1.765f;
double STEERING_FACTOR=18.0f;

double x=0.0;
double y=0.0;
double theta=0.0;

double lastStamp=0.0;

ros::Publisher odom_pub;

void ackermanDriveCalculus(const robotics_project::floatStamped::ConstPtr& Vl, 
                           const robotics_project::floatStamped::ConstPtr& Vr, 
                           const robotics_project::floatStamped::ConstPtr& steer){//callback function. ConstPtr is a pointer to the data structure we receive
    
    //computing velocity
    double normalizedSteeringAngle=steer->data/STEERING_FACTOR;
    double frontVelocity=(Vl->data+Vr->data)/2.0f;
    double angularVelocity=frontVelocity*sin(normalizedSteeringAngle)/WHEEL_BASELINE;
    double rearVelocity=(angularVelocity*WHEEL_BASELINE)/tan(normalizedSteeringAngle);
    double Vx=rearVelocity;
    double Vy=0.0;

    ROS_INFO("V: [%f], angularVelocity: [%f]", rearVelocity,angularVelocity);

    //computing dt
    double timeSpan;
    if (lastStamp==0.0)
    {
      timeSpan=0.0;
    }
    else
    {
      timeSpan= Vl->header.stamp.toSec()- lastStamp;
    }
    lastStamp=Vl->header.stamp.toSec();
    

    //exact integration
    theta=theta+angularVelocity*timeSpan;
    x=x+rearVelocity*timeSpan*cos(theta);
    y=y+rearVelocity*timeSpan*sin(theta);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);


    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id="odom";
    odom_trans.child_frame_id="base_footprint";
    odom_trans.transform.translation.x=x;
    odom_trans.transform.translation.y=y;
    odom_trans.transform.translation.z=0.0;
    odom_trans.transform.rotation=odom_quat;

    //prepare odom message
    nav_msgs::Odometry odom;
    odom.header.stamp=ros::Time::now();
    odom.header.frame_id="odom";

    odom.pose.pose.position.x=x;
    odom.pose.pose.position.y=y;
    odom.pose.pose.position.z=0.0;
    odom.pose.pose.orientation=odom_quat;

    odom.child_frame_id="base_footprint";
    odom.twist.twist.linear.x=Vx;
    odom.twist.twist.linear.y=Vy;
    odom.twist.twist.linear.z=0.0;

    odom.twist.twist.angular.x=0.0;
    odom.twist.twist.angular.y=0.0;
    odom.twist.twist.angular.z=angularVelocity;

    //publish odom
    odom_pub.publish(odom);

    static tf::TransformBroadcaster br;
    tf::Transform transform;//ros structure for tf
    transform.setOrigin( tf::Vector3(x, y, 0) );//set the origin in a 3d space (Vector3). As the turtle works in a 2d field, the 3rd value is set to 0
    tf::Quaternion q;
    q.setRPY(0, 0, theta);//2d spaces => 1 value of the quaternion
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "car"));//boradcast the transformation

}

int main(int argc, char **argv){
  	
	ros::init(argc, argv, "ackerman_node");

	ros::NodeHandle n;
    odom_pub=n.advertise<nav_msgs::Odometry>("odom",60);
    ROS_INFO("I'm alive");

  	message_filters::Subscriber<robotics_project::floatStamped> subSpeedL(n, "speedL_stamped", 1);
    message_filters::Subscriber<robotics_project::floatStamped> subSpeedR(n, "speedR_stamped", 1);
    message_filters::Subscriber<robotics_project::floatStamped> subSteer(n, "steer_stamped", 1);
    
    typedef message_filters::sync_policies::ApproximateTime <robotics_project::floatStamped, robotics_project::floatStamped, robotics_project::floatStamped> MySyncPolicy;//synchronizer with approximation
  
  
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subSpeedL, subSpeedR, subSteer);
    sync.registerCallback(boost::bind(&ackermanDriveCalculus, _1, _2, _3));
    
    //message_filters::TimeSynchronizer<robotics_project::floatStamped, robotics_project::floatStamped, robotics_project::floatStamped> sync(subSpeedL, subSpeedR, subSteer, 10);
    //sync.registerCallback(boost::bind(&ackermanDriveCalculus, _1, _2, _3));
  
  	ros::spin();

    return 0;
}
