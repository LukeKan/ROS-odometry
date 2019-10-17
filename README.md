# Robotics project 2019 (ROS)
First project for the course "Robotics" of Politecnico di Milano, a.y. 2018/2019.
Given a .bag file containing the records of an autonomous car moving around, the main task of the project requires the computation of the car odometry 
through the Ackerman model and the Differential drive kinematics.
It is possible to **dynamically reconfigure** and reset the 3 DoF of the car (x ,y, theta).
In order to switch between the two kinds of odometry, it is possible to use the dynamic_reconfigure node.
The result of the odometry computation is packed inside the Odom message, which contains:
1.nav_msgs/Odometry for the odometry values;
2.string for the kind of odometry.
The program is run through the ROS middleware.

#Description of the files inside the archive:

-CMakeLists.txt : 
  the file contains:
  1.the packages required by the node:
      -nav_msgs, used for the Odometry class;
      -tf, used to publish the tf;
      -message_generation, necessary to create and use custom messages;
      -message_filters, used to synchronize the speedL_stamped, speedR_stamped and steer_stamped topics coming from the bag file.
      -dynamic_reconfigure, necessary to implement the dynamic reconfiguration of the x,y coordinates and to switch the Odometry type.
  2.the custom messages reference and dependencies;
  3.the dynamic reconfigure file reference;
  4.the node dependencies for custom messages and message filters.

-package.xml : the file contains all the dependencies necessary for the node:
    -message_filters
    -custom_messages
    -dynamic_reconfigure

-cfg/Parameters.cfg : this file is used for the dynamic reconfiguration of the node for x,y and Odometry type.
  It contains an enum parameter for the Odometry type and two int parameters for the x,y coordinates.

-msg/floatStamped.msg :
  file that declares the type of steer_stamped, speedL_stamped,speedR_stamped.

-msg/customOdometry.msg :
  file used to declare a custom structure made of a string odom_type that contains the Odometry type currently computed and 
  a nav_msgs/Odometry that contains the Odometry computed.

-src/odometry_node.cpp :
  by receiving data from the bag files, this node computes the odometry and publishes it in:
     -the /odom topic as nav_msgs/Odometry;
     -the /tf topic as tf;
      -the /custom_odom topic as customOdometry (custom message).
  Also, it provides the dynamic reconfiguration of the odom type and x,y coordinates using the Parameters.cfg parameters.

#The workflow
1.main:
  - Init of the node as "odometry_node";  
  - Subscribe to the topics speedL_stamped, speedR_stamped and steer_stamped and synchronize them inside a message_filter with ApproximatedTimestamp as their stamp is not very accurate;
  - Binds the message_filter to the function odometryCalculus;
  - Create and bind the dynamic_reconfigure server to the function dinamicReconfigure.

2.odometryCalculus:
    -checks if the odom_type is either ackerman or differential and computes with the specified odom_type the rearWheelSpeed and the angularSpeed of the car;
    -integrates the rearWheelSpeed and angularSpeed with the Runge-Kutta approximation in order to get x,y,theta;
    -creates the Odometry variable and adds in it the linear and angular speed and the pose of the car;
    -creates the customOdometry variable and adds in it the Odometry variable and the odom_type string;
    -creates the tf variable and adds the pose;
    -publishes the three messages.

3.dinamicReconfigure:
    -updates the x,y and odom_type with the reconfiguration parameters received.


#Tf and odometry parameters
    -frame_id: odom
    -child_frame_id: car

#dynamic_reconfigure parameters:
    -x, int for the x coordinate;
    -y, int for the y coordinate;
    -string_type, enum for the odometry type, can be either "ackerman" or "differential".

#How to run the project:

    -after compiling (catkin_make), the odometry_node will be created;
    -run the node as follows: rosrun robotics_project odometry_node
    -the node will give an acknowledgement saying "I'm alive";
    -run the bag file -> the node will start computing the rearWheelSpeed and angularSpeed;
    -the results can be shown in rviz by adding the tf and odometry tabs and configuring the fixed frame as "odom" and the Odometry topic as "odom".

#Notes:
I decided not to create directly a reset parameter for x,y, as it is easily feasable by setting the x,y int parameters to 0,0.
