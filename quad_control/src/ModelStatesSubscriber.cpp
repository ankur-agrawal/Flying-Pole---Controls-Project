#include "ModelStatesSubscriber.h"

void ModelStatesSubscriber::modelCB(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  gazebo_msgs::ModelState quadrotor;
  gazebo_msgs::ModelState pendulum;

  // quadrotor.pose.size()=2;
  // std::cout << msg->pose[1].position.z << '\n';
  // quadrotor.pose.assign(1,0.0);
  // quadrotor.twist.assign(1,0.0);
  for (int i=0;i<msg->pose.size();i++)
  {
    if (msg->name[i]=="quadrotor")
    {
      quadrotor.model_name=msg->name[i];
      quadrotor.pose=msg->pose[i];
      quadrotor.twist=msg->twist[i];
    }
    if (msg->name[i]=="pendulum")
    {
      pendulum.model_name=msg->name[i];
      pendulum.pose=msg->pose[i];
      pendulum.twist=msg->twist[i];
    }
    hector_uav_msgs::MotorCommand motor;
    ros::Rate loop_rate(1000);
    float force;
    motor.header.frame_id = "base_link";
    motor.header.stamp=ros::Time::now();
    motor.force.assign(4, 0.0);
    motor.torque.assign(4, 0.0);
    motor.frequency.clear();
    motor.voltage.assign(4, 0.0);
    int iter=0;
    double t=0;
    double V=0;

    V=(80*1.477*(1-msg->pose[1].position.z)+30*1.477*(0-msg->twist[1].linear.z)+7.051760)/4;
    std::cout << "V="<< V << '\n';
    motor.force[0] =  force;
    motor.force[1] =  force;
    motor.force[2] =  force;
    motor.force[3] =  force;

    motor.voltage[0] = V;
    motor.voltage[1] = V;
    motor.voltage[2] = V;
    motor.voltage[3] = V;

    motor.torque[0] = 0 ;
    motor.torque[1] = 0 ;
    motor.torque[2] = 0 ;
    motor.torque[3] = 0 ;
    // std::cout << V << '\n';
    motor_pub.publish(motor);
    // loop_rate.sleep();

  }
  // std::cout << pendulum.pose.position << '\n';

}
int main(int argc, char**argv)
{

  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle n;

  ModelStatesSubscriber obj(n);

  ros::spin();

}
