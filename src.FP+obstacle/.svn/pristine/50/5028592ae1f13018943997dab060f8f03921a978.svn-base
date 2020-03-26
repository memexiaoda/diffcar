/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <cstdio>
#include "tf/transform_broadcaster.h"
#include <std_msgs/Bool.h>

class TransformSender
{
public:
  ros::NodeHandle node_;
  //constructor
  TransformSender(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string& frame_id, const std::string& child_frame_id)
  { 
    tf::Quaternion q;
    q.setRPY(roll, pitch,yaw);
    transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x,y,z)), time, frame_id, child_frame_id );
    stop_static_sub = node_.subscribe("/stop_static_transform",1,&TransformSender::static_transform_stopcallback,this);
    static_transform = true;
  };
  TransformSender(double x, double y, double z, double qx, double qy, double qz, double qw, ros::Time time, const std::string& frame_id, const std::string& child_frame_id) :
    transform_(tf::Transform(tf::Quaternion(qx,qy,qz,qw), tf::Vector3(x,y,z)), time, frame_id, child_frame_id),static_transform(true),stop_static_sub(node_.subscribe("/stop_static_transform",1,&TransformSender::static_transform_stopcallback,this)){};
  //Clean up ros connections
  ~TransformSender() {}

  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;

  ros::Subscriber stop_static_sub;

  void static_transform_stopcallback(const std_msgs::Bool static_transform_stop){
	  static_transform = static_transform_stop.data;
          ROS_ERROR("static_transform listen %u \n",static_transform);
  }

  bool static_transform;
  // A function to call to send data periodically
  void send (ros::Time time) {
    transform_.stamp_ = time;
    broadcaster.sendTransform(transform_);
  };

private:
  tf::StampedTransform transform_;

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv,"static_transform_publisher", ros::init_options::AnonymousName);
  ros::NodeHandle nd_;


  if(argc == 11)
  {
    ros::Duration sleeper(atof(argv[10])/1000.0);

    if (strcmp(argv[8], argv[9]) == 0)
      ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[8], argv[9]);

    TransformSender tf_sender(atof(argv[1]), atof(argv[2]), atof(argv[3]),
                              atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]),
                              ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                              argv[8], argv[9]);

   
    while(tf_sender.node_.ok() && tf_sender.static_transform)
    {
      ros::spinOnce();
      tf_sender.send(ros::Time::now() + sleeper);
      ROS_DEBUG("Sending transform from %s with parent %s\n", argv[8], argv[9]);
      sleeper.sleep();
      ROS_WARN("tf_sender.static_transform %u \n",tf_sender.static_transform);
    }
    ros::spin();
    return 0;
  } 
  else if (argc == 10)
  {
    ros::Duration sleeper(atof(argv[9])/1000.0);

    if (strcmp(argv[7], argv[8]) == 0)
      ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[7], argv[8]);

    TransformSender tf_sender(atof(argv[1]), atof(argv[2]), atof(argv[3]),
                              atof(argv[4]), atof(argv[5]), atof(argv[6]),
                              ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                              argv[7], argv[8]);



    while(tf_sender.node_.ok() && tf_sender.static_transform)
    {
      ros::spinOnce();
      tf_sender.send(ros::Time::now() + sleeper);
      ROS_DEBUG("Sending transform from %s with parent %s\n", argv[7], argv[8]);
      sleeper.sleep();
    }
    ros::spin();
    return 0;

  }
  else
  {
    printf("A command line utility for manually sending a transform.\n");
    printf("It will periodicaly republish the given transform. \n");
    printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds) \n");
    printf("OR \n");
    printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds) \n");
    printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
    printf("of the child_frame_id.  \n");
    ROS_ERROR("static_transform_publisher exited due to not having the right number of arguments");
    ros::spin();
    return -1;
  }


};

