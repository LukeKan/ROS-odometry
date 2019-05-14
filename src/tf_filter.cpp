#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <tf/transform_broadcaster.h>

class tf_sub_pub
{
public:
  	tf_sub_pub(){
  	sub = n.subscribe("/turtle1/pose", 1000, &tf_sub_pub::callback, this);
}


void callback(const turtlesim::Pose::ConstPtr& msg){
tf::Transform transform;//ros structure for tf
transform.setOrigin( tf::Vector3(msg->x, msg->y, 0) );//set the origin in a 3d space (Vector3). As the turtle works in a 2d field, the 3rd value is set to 0
tf::Quaternion q;
q.setRPY(0, 0, msg->theta);//2d spaces => 1 value of the quaternion
transform.setRotation(q);
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtle"));//boradcast the transformation
}

private:
ros::NodeHandle n; 
tf::TransformBroadcaster br;
ros::Subscriber sub;
};


int main(int argc, char **argv)
{
 ros::init(argc, argv, "tf_filter");
 tf_sub_pub my_tf_sub_bub;
 ros::spin();
 return 0;
}
