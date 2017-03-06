#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <Eigen/Dense>
#include <std_msgs/Bool.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include <hector_quadrotor_controller/quadrotor_interface.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
using namespace Eigen;
using namespace std;

class motor_controller{
private:
//    ros::Publisher motor_pub;
  ros::Publisher motor_pub;
//
public:
  motor_controller(ros::NodeHandle n )
  {

     motor_pub = n.advertise<hector_uav_msgs::MotorCommand>("/command/motor",1000);
   }
   void MotorCommandPublisher();
 };
