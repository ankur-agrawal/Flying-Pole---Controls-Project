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
    double q0,q1,q2,q3;
    q0=msg->pose[1].orientation.w;
    q1=msg->pose[1].orientation.x;
    q2=msg->pose[1].orientation.y;
    q3=msg->pose[1].orientation.z;

    double phi, theta, psi;

    psi=atan2(2*(q0*q3-q1*q2),1-2*(q2*q2+q3*q3));
    phi=atan2(2*(q0*q1-q3*q2),1-2*(q1*q1+q2*q2));
    theta=asin(2*(q0*q2+q2*q3));

    // std::cout << phi << '\t' << theta << '\t' << psi << '\t'<< '\n';
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

    MatrixXd R(3,3);
    R << cos(theta)*cos(psi), -cos(theta)*sin(psi), sin(theta),
    cos(phi)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi)-sin(psi)*sin(phi)*sin(theta), -cos(theta)*sin(phi),
    sin(phi)*sin(psi)-cos(psi)*cos(phi)*sin(theta), sin(phi)*cos(psi)+sin(psi)*cos(phi)*sin(theta), cos(theta)*cos(phi);

    MatrixXd Angular_vel_w(3,1);
    MatrixXd Angular_vel_b(3,1);

    Angular_vel_w << msg->twist[1].angular.x,
                    msg->twist[1].angular.y,
                    msg->twist[1].angular.z;
    Angular_vel_b=R.inverse()*Angular_vel_w;
   //
   //
  //   MatrixXd A(12,12);
  //   MatrixXd B(12,4);
  //   MatrixXd X(12,1);
  //   MatrixXd X_d(12,1);
  //   MatrixXd K(4,12);
    MatrixXd F(4,1);
    MatrixXd u(4,1);
    MatrixXd F_c(4,4);
   //
  //   A << 0,0,0,1,0,0,0,0,0,0,0,0,
  //       0,0,0,0,1,0,0,0,0,0,0,0,
  //       0,0,0,0,0,1,0,0,0,0,0,0,
  //       0,0,0,0,0,0,0,9.81,0,0,0,0,
  //       0,0,0,0,0,0,-9.81,0,0,0,0,0,
  //       0,0,0,0,0,0,0,0,0,0,0,0,
  //       0,0,0,0,0,0,0,0,0,1,0,0,
  //       0,0,0,0,0,0,0,0,0,0,1,0,
  //       0,0,0,0,0,0,0,0,0,0,0,1,
  //       0,0,0,0,0,0,0,0,0,0,0,0,
  //       0,0,0,0,0,0,0,0,0,0,0,0,
  //       0,0,0,0,0,0,0,0,0,0,0,0;
   //
  //   B << 0,0,0,0,
  //       0,0,0,0,
  //       0,0,0,0,
  //       0,0,0,0,
  //       0,0,0,0,
  //       1,1,1,1,
  //       0,0,0,0,
  //       0,0,0,0,
  //       0,0,0,0,
  //       0.25/0.01152,0,-0.25/0.01152,0,
  //       0,0.25/0.01152,0,-0.25/0.01152,
  //       0.25/0.0218,-0.25/0.0218,0.25/0.0218,-0.25/0.0218;
   //
  //   K <<     -0.1667,    -2.2361,    1.8565,   -0.3370   ,-4.5117,    2.1356   ,33.6832,   -2.5259   ,12.3522,    7.1800   ,-0.5473,    3.9577,
  //   2.4634,    0.0000,    1.2469,    4.9712 ,   0.0000  ,  1.4329 ,  -0.0000  , 37.1236 , -15.4166  , -0.0000 ,   7.9216  , -4.9322,
  //  -0.1667,    2.2361,    1.8565,   -0.3370  ,  4.5117 ,   2.1356  ,-33.6832 ,  -2.5259  , 12.3522 ,  -7.1800  , -0.5473 ,   3.9577,
  //  -1.9687,   -0.0000,    1.2459,   -3.9716   ,-0.0000,    1.4319   ,-0.0000,  -29.6434  ,-21.3816,   -0.0000   ,-6.3120,   -6.8185;
   //
   //
  //   X << quadrotor.pose.position.x,
  //       quadrotor.pose.position.y,
  //       quadrotor.pose.position.z,
  //       quadrotor.twist.linear.x,
  //       quadrotor.twist.linear.y,
  //       quadrotor.twist.linear.z,
  //       phi,
  //       theta,
  //       psi,
  //       Angular_vel_b(0,0),
  //       Angular_vel_b(1,0),
  //       Angular_vel_b(2,0);
   //
  //   X_d << 1,
  //         0,
  //         1,
  //         0,
  //         0,
  //         0,
  //         0,
  //         0,
  //         0,
  //         0,
  //         0,
  //         0;
   //
  //   F_e << 7.05176,
  //         7.05176,
  //         7.05176,
  //         7.05176;
  //   F=-K*1.477*(X-X_d)+F_e;
   //
  //   cout << msg->twist[1].angular.z<< '\n';


    // V=(80*1.477*(0.5-msg->pose[1].position.z)+30*1.477*(0-msg->twist[1].linear.z)+7.051760)/4;
    double xdd_c=0,ydd_c=0,zdd_c=0,u1=0,theta_c=0,phi_c=0, u2=0, u3=0, u4=0;

    xdd_c=0.6*(-1-msg->pose[1].position.x)+1.2*(0-msg->twist[1].linear.x);
    ydd_c=0.6*(-1.5-msg->pose[1].position.y)+1.2*(0-msg->twist[1].linear.y);
    zdd_c=100*(1-msg->pose[1].position.z)+90*(0-msg->twist[1].linear.z);//100,80
    u(0,0)=1.477*9.81+1.477*zdd_c;

    phi_c=-ydd_c/9.81;
    theta_c=xdd_c/9.81;

    std::cout << msg->pose[1].position.x << '\t' << msg->pose[1].position.y << '\t' << msg->pose[1].position.z << '\t'  << '\n';
    // std::cout << phi_c << '\t' << phi << '\n';
    u(1,0)=0.3*(phi_c-phi)+0.12*(0-Angular_vel_b(0,0));///0.1,0.04
    u(2,0)=0.3*(theta_c-theta)+0.12*(0-Angular_vel_b(1,0));
    u(3,0)=1*(0.1-psi)+0.5*(0-Angular_vel_b(2,0));

    F_c << 1,1,1,1,
          1,0,-1,0,
          0,1,0,-1,
          1,-1,1,-1;

    F=F_c.inverse()*u;

    // cout<<F(0,0) << '\t' << F(1,0) << '\t'<< F(2,0) << '\t' << F(3,0) << '\t'<< '\n';
    // cout<< u(0,0) << '\t' << u(1,0) << '\t'<< u(2,0) << '\t' << u(3,0) << '\t'<< '\n';

    // std::cout << msg->pose[1].position.x << '\t' << psi << '\n';
    // V=7.051760/4;
    // std::cout << "V="<< V << '\n';

    double nominal_thrust_per_motor=u(0,0)/4;
    double nominal_torque_per_motor=u(3,0)/4;
    motor.force[0] =  nominal_thrust_per_motor - u(2,0) / 2.0 / 0.275;
    motor.force[1] =  nominal_thrust_per_motor - u(1,0) / 2.0 / 0.275;
    motor.force[2] =  nominal_thrust_per_motor + u(2,0) / 2.0 / 0.275;
    motor.force[3] =  nominal_thrust_per_motor + u(1,0) / 2.0 / 0.275;

    // motor.voltage[0] = V;
    // motor.voltage[1] = V;
    // motor.voltage[2] = V;
    // motor.voltage[3] = V;

    motor.voltage[0] = motor.force[0] / 0.559966216 + nominal_torque_per_motor / 7.98598e-3;
    motor.voltage[1] = motor.force[1] / 0.559966216 - nominal_torque_per_motor / 7.98598e-3;
    motor.voltage[2] = motor.force[2] / 0.559966216 + nominal_torque_per_motor / 7.98598e-3;
    motor.voltage[3] = motor.force[3] / 0.559966216 - nominal_torque_per_motor / 7.98598e-3;

    if (motor.voltage[0] < 0.0) motor.voltage[0] = 0.0;
    if (motor.voltage[1] < 0.0) motor.voltage[1] = 0.0;
    if (motor.voltage[2] < 0.0) motor.voltage[2] = 0.0;
    if (motor.voltage[3] < 0.0) motor.voltage[3] = 0.0;

    // std::cout << motor.voltage[0] << '\t' << motor.voltage[1] << '\t' << motor.voltage[2] << '\t' << motor.voltage[3] << '\t' << '\n';

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
