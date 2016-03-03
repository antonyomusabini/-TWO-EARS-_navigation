#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "roboteq_msgs/Feedback.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

#define Wheelbase (0.335)
#define WHEEL_RADIUS (0.2/2.0)
#define CPR_CODEUR 2048                            // tics / tr
#define GEAR_RATIO 11.1                              //

#define PI         (3.1415926)

float right_speed=0.0, right_position=0.0, left_speed=0.0, left_position=0.0;
float right_speed_ =0.0, left_speed_=0.0;
float right_linear_speed = 0.0, left_linear_speed = 0.0;
float x=0.0,y=0.0,th=0.0, dt=0.0, dl=0.0, dth=0.0;

geometry_msgs::Point position_;
geometry_msgs::Twist twist_;
double yaw_;

ros::Time current_time, last_time;
ros::Time last_cmd_vel_time;

void rightEncoderCallback(const roboteq_msgs::Feedback::ConstPtr& _feedback)
{
  right_speed = _feedback->measured_velocity;
  right_position = _feedback->measured_position;
}

void leftEncoderCallback(const roboteq_msgs::Feedback::ConstPtr& _feedback)
{
  left_speed = _feedback->measured_velocity;
  left_position = _feedback->measured_position;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_listener");
  ros::NodeHandle n;
  ros::Subscriber sub_right=n.subscribe("/right/feedback", 5, rightEncoderCallback);
  ros::Subscriber sub_left=n.subscribe("/left/feedback", 5, leftEncoderCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Rate loop_rate(10);
  tf::TransformBroadcaster odom_broadcaster;

  float delta_encoder_left, delta_encoder_right, raw_delta_x, raw_delta_theta;
  float delta_x, delta_y, delta_th;

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok())
  { 
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    left_speed_ = -left_speed / GEAR_RATIO;
    right_speed_ = right_speed / GEAR_RATIO;
    left_linear_speed = left_speed_ * WHEEL_RADIUS;
    right_linear_speed = right_speed_ * WHEEL_RADIUS;
    position_.x += cos(yaw_) * twist_.linear.x * dt;
    position_.y += sin(yaw_) * twist_.linear.x * dt;
    yaw_ += -1*twist_.angular.z * dt;                         // MODIF ANT : -1
    twist_.linear.x = (left_linear_speed + right_linear_speed) / 2;
    twist_.angular.z = (right_linear_speed - left_linear_speed) / Wheelbase;
    /****************************************************/
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw_);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = position_.x;
    odom_trans.transform.translation.y = position_.y; 
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = position_.x;
    odom.pose.pose.position.y = position_.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = twist_.linear.x;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = twist_.linear.z;

    //publish the message
    odom_pub.publish(odom);
    ros::spinOnce();

    last_time = current_time;
    loop_rate.sleep();
  }

  return 0;
}
