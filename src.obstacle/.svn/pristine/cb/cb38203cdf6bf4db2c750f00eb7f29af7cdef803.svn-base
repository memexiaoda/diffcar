/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class TurtlebotTeleop
{
public:
  TurtlebotTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
  void controlstyleCallback(const std_msgs::String& controlstyle);

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_,joy_switch_axis_open_,joy_switch_axis_close_;
  int axis_linear_desc_,axis_angular_desc_,axis_linear_add_,axis_angular_add_;
  double axis_linear_max_,axis_angular_max_;
  double l_scale_, a_scale_,vel_step_l_,vel_step_a_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Publisher control_style_pub;
  std::string control_style;
  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool joy_switch_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

TurtlebotTeleop::TurtlebotTeleop():
  ph_("~"),
  linear_(1),
  angular_(0),
  deadman_axis_(3),
  joy_switch_axis_open_(4),
  joy_switch_axis_close_(5),
  l_scale_(0.3),
  a_scale_(0.9),
  axis_linear_max_(1.5),
  axis_angular_max_(0.6),
  axis_linear_add_(15),
  axis_angular_add_(13),
  axis_linear_desc_(16),
  axis_angular_desc_(14),
  vel_step_l_(0.1),
  vel_step_a_(0.1)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("axis_joy_switch_open", joy_switch_axis_open_, joy_switch_axis_open_);
  ph_.param("axis_joy_switch_close", joy_switch_axis_close_, joy_switch_axis_close_);
  ph_.param("axis_linear_max", axis_linear_max_, axis_linear_max_);
  ph_.param("axis_angular_max", axis_angular_max_, axis_angular_max_);
  ph_.param("axis_linear_add", axis_linear_add_, axis_linear_add_);
  ph_.param("axis_angular_add", axis_angular_add_, axis_angular_add_);
  ph_.param("axis_linear_desc", axis_linear_desc_, axis_linear_desc_);
  ph_.param("axis_angular_desc", axis_angular_desc_, axis_angular_desc_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("vel_step_l", vel_step_l_, vel_step_l_);
  ph_.param("vel_step_a", vel_step_a_, vel_step_a_);
  //遥控手柄开关
  joy_switch_pressed_ = false;
  zero_twist_published_ = false;
  control_style = "auto";
  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel_manual", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &TurtlebotTeleop::joyCallback, this);
  control_style_pub = ph_.advertise<std_msgs::String>("/control_style", 1,true);
  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::publish, this));
}


void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];

  last_published_ = vel;

  if(joy->buttons[joy_switch_axis_open_]){
	  if(!joy_switch_pressed_){
		  joy_switch_pressed_ = true;
		  std_msgs::String ctrl_style;
		  control_style = "manual";
		  ctrl_style.data = "manual";
		  control_style_pub.publish(ctrl_style);
	  }
  }

  if(joy->buttons[joy_switch_axis_close_]){
  	  if(joy_switch_pressed_){
  		  joy_switch_pressed_ = false;
  		  std_msgs::String ctrl_style;
  		  control_style = "auto";
  		  ctrl_style.data = "auto";
  		  control_style_pub.publish(ctrl_style);
  	  }
  }

  if(joy->buttons[deadman_axis_]){
	  vel.linear.x = 0.0;
  }

  if(joy->buttons[axis_linear_add_]){
	  if(l_scale_ < axis_linear_max_)
	  {
		  l_scale_ +=  vel_step_l_;
	  }
	  ROS_INFO("l_scale_=%f,joy->buttons[axis_linear_add_]=%d",l_scale_,joy->buttons[axis_linear_add_]);
  }

  if(joy->buttons[axis_linear_desc_]){
	  if(l_scale_ >= vel_step_l_)
		  l_scale_ -=  vel_step_l_;
	  ROS_INFO("l_scale_=%f,joy->buttons[axis_linear_desc_]=%d",l_scale_,joy->buttons[axis_linear_desc_]);
  }

  if(joy->buttons[axis_angular_add_]){
	  if(a_scale_ < axis_angular_max_)
		  a_scale_ += vel_step_a_;
	  ROS_INFO("a_scale_=%f,joy->buttons[axis_angular_add_]=%d",a_scale_,joy->buttons[axis_angular_add_]);
  }

  if(joy->buttons[axis_angular_desc_]){
	  if(a_scale_ > vel_step_a_)
		  a_scale_ -= vel_step_a_;
	  ROS_INFO("a_scale_=%f,joy->buttons[axis_angular_desc_]=%d",a_scale_,joy->buttons[axis_angular_desc_]);
  }
}

void TurtlebotTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (joy_switch_pressed_)
  {
	if(control_style == "manual")
		vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }
  else if(!joy_switch_pressed_ && !zero_twist_published_)
  {
    if(control_style == "manual")
    	vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_teleop");
  TurtlebotTeleop turtlebot_teleop;

  ros::spin();
}
