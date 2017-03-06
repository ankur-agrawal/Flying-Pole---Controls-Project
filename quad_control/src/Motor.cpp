#include "Motor.h"

void motor_controller::MotorCommandPublisher()
{
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
    while (true)
  {
    iter=iter+1;
    if (iter<10000)
    {
      t=0;
      V=0;
    }
    else
    {
      V=7.051765*1.477*(4.5*t*t-12*t+15.81)/(4*3.6223);
      if (t>2)
      {
        V=7.051760;
      }

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
      loop_rate.sleep();
      std::cout << t << '\n';
      t=t+0.001;
    }
  }
}
int main(int argc, char**argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle n;

  motor_controller obj(n);

  obj.MotorCommandPublisher();
}
