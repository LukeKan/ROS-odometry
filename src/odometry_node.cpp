#include "ros/ros.h"
#include "robotics_project/floatStamped.h"
#include "robotics_project/customOdom.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <robotics_project/ParametersConfig.h>



const double WHEEL_BASELINE=1.3;
const double REAR_FRONT_DISTANCE=1.765;
const double STEERING_FACTOR=18.0;
const double PI=3.14159265;

double x=0.0;
double y=0.0;
double theta=0.0;

double lastStamp=0.0;
std::string odom_type="ackerman";

ros::Publisher odom_pub;
ros::Publisher odomTest;

void ackermanDriveCalculus(const robotics_project::floatStamped::ConstPtr& Vl, 
                           const robotics_project::floatStamped::ConstPtr& Vr, 
                           const robotics_project::floatStamped::ConstPtr& steer){//callback function. ConstPtr is a pointer to the data structure we receive
    
    double rearVelocity;
    double angularVelocity;
    double Vx;
    double Vy; 

    //computing velocity
    if(odom_type=="ackerman"){
      double normalizedSteeringAngle=steer->data/STEERING_FACTOR;   
      rearVelocity=(Vl->data+Vr->data)/2.0;
      angularVelocity=rearVelocity*(tan(normalizedSteeringAngle*PI/180));
      angularVelocity=angularVelocity/REAR_FRONT_DISTANCE;
      Vx=rearVelocity;
      Vy=0.0;
    } else if(odom_type=="differential")
    {
      rearVelocity = (Vl->data+Vr->data)/2.0f;

      angularVelocity = (Vr->data - Vl->data)/WHEEL_BASELINE;
      Vx=rearVelocity*cos(theta);
      Vy=rearVelocity*sin(theta);
    }
    

    

    ROS_INFO("V: [%f], w: [%f]", rearVelocity,angularVelocity);


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

    //runge-kutta integration
    x+=rearVelocity*timeSpan*cos(theta+(angularVelocity*timeSpan)/2);
    y+=rearVelocity*timeSpan*sin(theta+(angularVelocity*timeSpan)/2);
    theta+=angularVelocity*timeSpan;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);


    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id="odom";
    odom_trans.child_frame_id="car";
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

    odom.child_frame_id="car";
    odom.twist.twist.linear.x=Vx;
    odom.twist.twist.linear.y=Vy;
    odom.twist.twist.linear.z=0.0;

    odom.twist.twist.angular.x=0.0;
    odom.twist.twist.angular.y=0.0;
    odom.twist.twist.angular.z=angularVelocity;

    //publish custom odom
    robotics_project::customOdom custom_odom;
    custom_odom.odometry=odom;
    custom_odom.odometry_type=odom_type;
    odomTest.publish(odom);
    odom_pub.publish(custom_odom);

    static tf::TransformBroadcaster br;
    tf::Transform transform;//ros structure for tf
    transform.setOrigin( tf::Vector3(x, y, 0) );//set the origin in a 3d space (Vector3). As the turtle works in a 2d field, the 3rd value is set to 0
    tf::Quaternion q;
    q.setRPY(0, 0, theta);//2d spaces => 1 value of the quaternion
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "car"));//boradcast the transformation

}

int main(int argc, char **argv){
  	
	ros::init(argc, argv, "odometry_node");

	ros::NodeHandle n;
    odom_pub=n.advertise<robotics_project::customOdom>("custom_odom",60);
    odomTest=n.advertise<nav_msgs::Odometry>("odom",60);
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
