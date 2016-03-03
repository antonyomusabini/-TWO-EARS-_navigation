#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

#include <fstream>
#include <iostream>
#include <string>

using namespace std;

class KemarControl
{
public:
  KemarControl();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_, mapping_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber coord_sub;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool mapping_pressed_;
  ros::Timer timer_;

};

KemarControl::KemarControl():
  ph_("~"),
  linear_(1),
  angular_(0),
  deadman_axis_(4),
  mapping_(2),
  l_scale_(0.3),
  a_scale_(0.9)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("mapping", mapping_, mapping_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  deadman_pressed_ = false;
  mapping_pressed_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &KemarControl::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&KemarControl::publish, this));
  
}

void KemarControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  if((vel.angular.z<0.05)&&(vel.angular.z>-0.05))
	vel.angular.z = 0.0;
  if((vel.linear.x<0.05)&&(vel.linear.x>-0.05))
	vel.linear.x = 0.0;
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
  mapping_pressed_ = joy->buttons[mapping_];

}

void KemarControl::publish()
{
  geometry_msgs::Twist vel_lock;
  boost::mutex::scoped_lock lock(publish_mutex_);
  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
  }
  else
  {
    vel_lock.angular.z = 0.0;
    vel_lock.linear.x = 0.0;
    vel_pub_.publish(vel_lock);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kemar_control");
  KemarControl kemar_control;

  ros::spin();
}
