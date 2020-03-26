/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include "../include/base_local_planner/trajectory_planner_ros.h"

#include <sys/time.h>
#include <boost/tokenizer.hpp>

#include <eigen3/Eigen/Core>
#include <cmath>

//#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <string>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(base_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace base_local_planner {

  void TrajectoryPlannerROS::reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        //Avoid looping
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }
      tc_->reconfigure(config);
      reached_goal_ = false;
  }

  TrajectoryPlannerROS::TrajectoryPlannerROS() :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom"),_emergencyStop(false),_nh("TrajectoryPlannerROS"),debug_pid_(false) {}

  TrajectoryPlannerROS::TrajectoryPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom"),_emergencyStop(false),_nh("TrajectoryPlannerROS"),debug_pid_(false) {

      //initialize the planner
      initialize(name, tf, costmap_ros);
  }

  void TrajectoryPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      //obstacle_pub = private_nh.advertise<geometry_msgs::Vector3>("obstacle_laser_high",1);
      attitude_pub = private_nh.advertise<std_msgs::String>("/pose_attitude", 1);
      //high_laser_state_pub = private_nh.advertise<std_msgs::String>("/high_laser_state", 1);
      dockstop_pub = private_nh.advertise<std_msgs::String>("/dock_stop", 1);
      control_style_pub = private_nh.advertise<std_msgs::String>("/control_style", 1);
      //obstacle_dis = 6.0;
      control_style = "auto";

      tf_ = tf;
      costmap_ros_ = costmap_ros;
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      double sim_time, sim_granularity, angular_sim_granularity;
      int vx_samples, vtheta_samples;
      double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
      bool holonomic_robot, dwa, simple_attractor, heading_scoring;
      double heading_scoring_timestep;
      //double max_vel_x, min_vel_x;
      double backup_vel;
      double stop_time_buffer;
      std::string world_model_type;
      rotating_to_goal_ = false;

      //initialize the copy of the costmap the controller will use
      costmap_ = costmap_ros_->getCostmap();

      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
      private_nh.param("prune_plan", prune_plan_, true);

      private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
      private_nh.param("Kp_move", Kp_move_, 20.0);
      private_nh.param("Kd_move", Kd_move_, 5.0);
      private_nh.param("Ki_move", Ki_move_, 0.0);
      private_nh.param("Ko_move", Ko_move_, 40.0);
      private_nh.param("Ka_move", Ka_move_, 1.0);
      private_nh.param("Kb_move", Kb_move_, 1.5);
      private_nh.param("debug_pid", debug_pid_, debug_pid_);
      //行走线段信息

      _DockPile = private_nh.getParam("/move_base/DockPile",_DockPile);
      _DockPoint = private_nh.getParam("/move_base/DockPoint",_DockPoint);
      _Carport = private_nh.getParam("/move_base/Carport",_Carport);

      private_nh.param<std::string>("DockPile", _DockPile, _DockPile);
      private_nh.param<std::string>("DockPoint", _DockPoint, _DockPoint);
      private_nh.param<std::string>("Carport", _Carport, _Carport);

      private_nh.param("start_point_id", _start_point_id, _DockPile);
      private_nh.param("end_point_id", _end_point_id, _DockPile);
      private_nh.param("task_point_id", _task_point_id, _DockPoint);
      private_nh.param("task_Line_id", _task_Line_id, std::string(_DockPile +"-"+ _DockPoint));

//      private_nh.param("LObstacleStop", LObstacleStop_, 0.7);
//      private_nh.param("LObstacleSlow", LObstacleSlow_, 1.7);
//      private_nh.param("LObstacle_front_sector", LObstacle_front_sector_, 45.0);
//      private_nh.param("LObstacleStop_left", LObstacleStop_left_, 0.7);
//      private_nh.param("LObstacle_left_sector", LObstacle_left_sector_, 45.0);
//      private_nh.param("LObstacleStop_right", LObstacleStop_right_, 0.7);
//      private_nh.param("LObstacle_right_sector", LObstacle_right_sector_, 45.0);

      private_nh.param("delay_time", delay_time_, 1.0);
      private_nh.param("delay_factor", delay_factor_, 0.086);
      private_nh.param("emergencyStop", _emergencyStop, false);

      private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
      private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
      private_nh.param("acc_lim_theta", acc_lim_theta_, 3.2);

      private_nh.param("stop_time_buffer", stop_time_buffer, 0.2);

      private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);
      private_nh.param("slowdown_goal_tolerance", slowdown_goal_tolerance_, 1.0);

      //Since I screwed up nicely in my documentation, I'm going to add errors
      //informing the user if they've set one of the wrong parameters
      if(private_nh.hasParam("acc_limit_x"))
        ROS_ERROR("You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(private_nh.hasParam("acc_limit_y"))
        ROS_ERROR("You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(private_nh.hasParam("acc_limit_th"))
        ROS_ERROR("You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      //Assuming this planner is being run within the navigation stack, we can
      //just do an upward search for the frequency at which its being run. This
      //also allows the frequency to be overwritten locally.
      std::string controller_frequency_param_name;
      if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
        sim_period_ = 0.05;
      else
      {
        double controller_frequency = 0;
        private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
        if(controller_frequency > 0)
          sim_period_ = 1.0 / controller_frequency;
        else
        {
          ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
          sim_period_ = 0.05;
        }
      }

      ROS_INFO("Sim period is set to %.2f", sim_period_);

      private_nh.param("sim_time", sim_time, 1.0);
      private_nh.param("sim_granularity", sim_granularity, 0.025);
      private_nh.param("angular_sim_granularity", angular_sim_granularity, sim_granularity);
      private_nh.param("vx_samples", vx_samples, 3);
      private_nh.param("vtheta_samples", vtheta_samples, 20);

      private_nh.param("path_distance_bias", pdist_scale, 0.6);
      private_nh.param("goal_distance_bias", gdist_scale, 0.8);
      private_nh.param("occdist_scale", occdist_scale, 0.01);

      bool meter_scoring;
      if ( ! private_nh.hasParam("meter_scoring")) {
        ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settins robust against changes of costmap resolution.");
      } else {
        private_nh.param("meter_scoring", meter_scoring, false);

        if(meter_scoring) {
          //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
          double resolution = costmap_->getResolution();
          gdist_scale *= resolution;
          pdist_scale *= resolution;
          occdist_scale *= resolution;
        } else {
          ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settins robust against changes of costmap resolution.");
        }
      }

      private_nh.param("heading_lookahead", heading_lookahead, 0.325);
      private_nh.param("oscillation_reset_dist", oscillation_reset_dist, 0.05);
      private_nh.param("escape_reset_dist", escape_reset_dist, 0.10);
      private_nh.param("escape_reset_theta", escape_reset_theta, M_PI_4);
      private_nh.param("holonomic_robot", holonomic_robot, true);
      private_nh.param("max_vel_x", max_vel_x, 1.0);
      private_nh.param("min_vel_x", min_vel_x, 0.1);
      private_nh.param("escape_vel", escape_vel_, -0.1);

      double max_rotational_vel;
      private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
      max_vel_th_ = max_rotational_vel;
      min_vel_th_ = -1.0 * max_rotational_vel;
      private_nh.param("min_in_place_rotational_vel", min_in_place_vel_th_, 0.1);
      private_nh.param("min_out_place_rotational_vel", min_out_place_vel_th_, 0.1);

      reached_goal_ = false;

      backup_vel = -0.1;

      if(private_nh.getParam("backup_vel", backup_vel))
        ROS_WARN("The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");

      //if both backup_vel and escape_vel are set... we'll use escape_vel
      private_nh.getParam("escape_vel", backup_vel);

      if(backup_vel >= 0.0)
        ROS_WARN("You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

      private_nh.param("world_model", world_model_type, std::string("costmap"));
      private_nh.param("dwa", dwa, true);
      private_nh.param("heading_scoring", heading_scoring, false);
      private_nh.param("heading_scoring_timestep", heading_scoring_timestep, 0.8);

      simple_attractor = false;

      //parameters for using the freespace controller
      double min_pt_separation, max_obstacle_height, grid_resolution;
      private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
      private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
      private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
      private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

      ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");

      world_model_ = new CostmapModel(*costmap_);

      std::vector<double> y_vels = loadYVels(private_nh);

      footprint_spec_ = costmap_ros_->getRobotFootprint();

      tc_ = new TrajectoryPlanner(*world_model_, *costmap_, footprint_spec_,
          acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,
          gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
          max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
          dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);

      ROS_WARN("++++++++++++++++++ max_vel_x=%.3f,min_vel_x=%.3f", max_vel_x, min_vel_x);

      map_viz_.initialize(name, global_frame_, boost::bind(&TrajectoryPlanner::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));
      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<BaseLocalPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<BaseLocalPlannerConfig>::CallbackType cb = boost::bind(&TrajectoryPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);

    } else {
      ROS_WARN("This planner has already been initialized, doing nothing");
    }
    ros::NodeHandle simple_nh("TrajectoryPlannerROS_obstacle");
    //laser_scan_sub = simple_nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, boost::bind(&TrajectoryPlannerROS::laserCB, this, _1));
    //laser_scan_sub = simple_nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, boost::bind(&TrajectoryPlannerROS::laserCB, this, _1));
    batteryInfo_sub = _nh.subscribe("/BatteryInfo",1,&TrajectoryPlannerROS::batteryInfo_rec, this);
    InpsectInfo_sub = _nh.subscribe("/InspectInfo",1,&TrajectoryPlannerROS::InpsectInfo_rec, this);
    equipmentInfo_sub = _nh.subscribe("/EquipmentInfo",1,&TrajectoryPlannerROS::equipmentInfo_rec, this);
  	//control_style_pub = _nh.subscribe<std_msgs::String>("/manual_req",1,boost::bind(&TrajectoryPlannerROS::controlstyleCB,this,_1));
    //arrived_sub_ =  _nh.subscribe<carbot_msgs::Taskarrived>("/task_arrived",1,boost::bind(&TrajectoryPlannerROS::task_arrived, this, _1));
    //arrived_sub_ =  _nh.subscribe("/task_arrived",1,&TrajectoryPlannerROS::task_arrived, this);
  	task_goal_sub = _nh.subscribe("/current_task", 1,&TrajectoryPlannerROS::task_goal_rec, this);
  	//delaytime_motion_sub = private_nh_new.subscribe("/delay_motion_time", 1,&TrajectoryPlannerROS::delay_motion_rec,this);
  	//private_nh_new.param("next_goal", p_next_goal, p_next_goal);
  }

  void TrajectoryPlannerROS::batteryInfo_rec(const carbot_msgs::BatteryInfo& batteryInfo)
  {
	  _batteryInfo = batteryInfo;
  }

  void TrajectoryPlannerROS::InpsectInfo_rec(const carbot_msgs::InspectInfo& inspectInfo)
  {
	  _inspectInfo = inspectInfo;
  }

  void TrajectoryPlannerROS::equipmentInfo_rec(const carbot_msgs::EquipmentInfo& equipmentInfo)
  {
	  _equipmentInfo = equipmentInfo;
  }

  void TrajectoryPlannerROS::task_goal_rec(const carbot_msgs::IDPose& nextgoal)
  {
	  _next_goal = nextgoal;
  }

  std::vector<double> TrajectoryPlannerROS::loadYVels(ros::NodeHandle node){
    std::vector<double> y_vels;
    std::string y_vel_list;
    if(node.getParam("y_vels", y_vel_list)){
      typedef boost::tokenizer< boost::char_separator<char> > tokenizer;
      boost::char_separator<char> sep("[], ");
      tokenizer tokens(y_vel_list, sep);
      for(tokenizer::iterator i = tokens.begin(); i != tokens.end(); i++){
        y_vels.push_back(atof((*i).c_str()));
      }
    }
    else{
      //if no values are passed in, we'll provide defaults
      y_vels.push_back(-0.3);
      y_vels.push_back(-0.1);
      y_vels.push_back(0.1);
      y_vels.push_back(0.3);
    }
    return y_vels;
  }

  TrajectoryPlannerROS::~TrajectoryPlannerROS() {
    //make sure to clean things up
    delete dsrv_;
    if(tc_ != NULL)
      delete tc_;
    if(world_model_ != NULL)
      delete world_model_;
  }

  bool TrajectoryPlannerROS::stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel){
    //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
    //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
    double vx = sign(robot_vel.getOrigin().x()) * std::max(0.0, (fabs(robot_vel.getOrigin().x()) - acc_lim_x_ * sim_period_));
    double vy = sign(robot_vel.getOrigin().y()) * std::max(0.0, (fabs(robot_vel.getOrigin().y()) - acc_lim_y_ * sim_period_));

    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

    //we do want to check whether or not the command is valid
    double yaw = tf::getYaw(global_pose.getRotation());
    bool valid_cmd = tc_->checkTrajectory(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw, 
        robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw, vx, vy, vth);

    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if(valid_cmd){
      ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vth;
      return true;
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool TrajectoryPlannerROS::rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel){
    double yaw = tf::getYaw(global_pose.getRotation());
    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
        std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_,
        std::min(-1.0 * min_in_place_vel_th_, ang_diff));

    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
    double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

    v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff)); 

    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

    // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
    v_theta_samp = v_theta_samp > 0.0
      ? std::min( max_vel_th_, std::max( min_in_place_vel_th_, v_theta_samp ))
      : std::max( min_vel_th_, std::min( -1.0 * min_in_place_vel_th_, v_theta_samp ));

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = tc_->checkTrajectory(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw, 
        robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw, 0.0, 0.0, v_theta_samp);

    ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

    if(valid_cmd)
    {
      cmd_vel.angular.z = v_theta_samp;
      //cmd_vel.angular.z = normalize_angle_me(cmd_vel.angular.z);
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;
    
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;
    //reset the at goal flag
    reached_goal_ = false;
    return true;
  }

  //计算当前点与路径直线间的距离
  double TrajectoryPlannerROS::cal_distance(geometry_msgs::Point cur_pos, geometry_msgs::Point pre_pose, geometry_msgs::Point next_pose)
  {
	  double a_x = next_pose.x- pre_pose.x;
	  double a_y = next_pose.y- pre_pose.y;
	  double ptl;
	  if(a_x == 0){
		  if(a_y >= 0){
			  ptl = cur_pos.x - next_pose.x;
		  }else{
			  ptl = next_pose.x - cur_pos.x;
		  }
	  }else{
		  double K = (next_pose.y - pre_pose.y)/(next_pose.x- pre_pose.x);
		  double B = (next_pose.x * pre_pose.y - pre_pose.x * next_pose.y)/(next_pose.x - pre_pose.x);
		  if(a_x > 0){
			  if ((-K * cur_pos.x + cur_pos.y - B) > 0)
				 ptl = -fabs(-K*cur_pos.x + cur_pos.y - B)/sqrt(pow(-K,2) + 1);
			 else
				 ptl = fabs(-K*cur_pos.x + cur_pos.y - B)/sqrt(pow(-K,2) + 1);
		  }else{
			  if ((-K*cur_pos.x + cur_pos.y - B) > 0)
				  ptl = fabs(-K*cur_pos.x + cur_pos.y - B)/sqrt(pow(-K,2) + 1);
			  else
				  ptl = -fabs(-K*cur_pos.x + cur_pos.y - B)/sqrt(pow(-K,2) + 1);
		  }
	  }
	  return ptl;
  }

  double TrajectoryPlannerROS::cal_path_rotation(geometry_msgs::Point cur_pos,geometry_msgs::Point next_pose){
	  double path_th;
	  //计算下一条路径直线和全局正方间的夹角 (规定正方向向量（xcur+1,ycur）)
	  double prodis = sqrt(pow(next_pose.x - cur_pos.x,2) + pow(next_pose.y - cur_pos.y,2));
	  double curdis = 1;
	  double dot_product = (next_pose.x - cur_pos.x);
	  double dif_product = (next_pose.y - cur_pos.y);
	  double cosm, sinm;

	  if (prodis==0){
		  cosm = cos(cur_pos.z);
		  sinm = sin(cur_pos.z);
	  }else{
		  cosm = dot_product / (prodis*curdis);
		  sinm = dif_product / (prodis*curdis);
	  }
	  if (cosm >= 0)
		  path_th = asin(sinm);
	  else{
		  if (sinm >=0)
			  path_th = M_PI - asin(sinm);
		  else
			  path_th = -M_PI - asin(sinm);
	  }
	  return path_th;
  }

  double TrajectoryPlannerROS::CacPid(PIDCONTRL pt){

	  double Perror = pt.Perror;
	  //double Kp=20.0, Kd=5.0, Ki=0.0, Ko=40.0;
	  double output = (Kp_move_*Perror + Kd_move_*(Perror - pt.PrevErr) + Ki_move_*pt.Ierror)/Ko_move_;
	  double MaxOutput = acc_lim_theta_; //#0.07           #PID最大误差值
	  pt.PrevErr = Perror;

	  if (output >= MaxOutput)
		  output = MaxOutput;
	  else if (output < -MaxOutput)
		  output = -MaxOutput;
	  else
		  pt.Ierror = pt.Ierror + Perror;
	  pt.Poutput = output;
	  //print"ControlThread::CacPid pt.output: ",pt.Poutput
	  return output;
  }

  double TrajectoryPlannerROS::normalize_angle_me(double angle){
      float res = angle;
      while (res > M_PI)
          res -= 2.0 * M_PI;
      while (res < -M_PI)
          res += 2.0 * M_PI;
      return res;
  }

  void TrajectoryPlannerROS::slowdown(geometry_msgs::Twist& cmd_vel){
	  if(cmd_vel.linear.x >= min_vel_x){
		  cmd_vel.linear.x = std::max( min_vel_x, cmd_vel.linear.x - acc_lim_x_ * 3);
		  //cmd_vel.angular.z = 0.0;
	  }
	  else{
		  //cmd_vel.linear.x = std::min(min_vel_x,cmd_vel.linear.x + acc_lim_x_);
		  //cmd_vel.angular.z = 0.0;
	  }
  }

  void TrajectoryPlannerROS::stopcar(geometry_msgs::Twist& cmd_vel){
	  if(cmd_vel.linear.x > 0.1){
		  cmd_vel.linear.x = std::max(0.0,cmd_vel.linear.x - acc_lim_x_*3);
		  ROS_WARN_DELAYED_THROTTLE(2,"stop speed . 1");
		  //cmd_vel.angular.z = 0.0;
	  }else if(cmd_vel.linear.x <= 0.1 && cmd_vel.angular.z > 0.1){
		  cmd_vel.linear.x = 0.0;
		  ROS_WARN_DELAYED_THROTTLE(2,"stop speed . 2");
	  }else if(cmd_vel.linear.x <= 0.1 && cmd_vel.angular.z <= 0.1){
		  cmd_vel.linear.x = 0.0;
		  cmd_vel.angular.z = 0.0;
		  ROS_WARN_DELAYED_THROTTLE(2,"stop speed . 3");
	  }else{
		  cmd_vel.linear.x = 0.0;
		  cmd_vel.angular.z = 0.0;
		  ROS_WARN_DELAYED_THROTTLE(2,"stop speed . 4");
	  }
	  if(cmd_vel.angular.z < 0.02){
		  cmd_vel.angular.z = 0.0;
	  }
  }

  bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel/*,double obstacle_dis*/){
	  if (! isInitialized()) {
		  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		  return false;
	  }

	  std::vector<geometry_msgs::PoseStamped> local_plan;
	  tf::Stamped<tf::Pose> global_pose;
	  if (!costmap_ros_->getRobotPose(global_pose)) {
		  return false;
	  }

	  std::vector<geometry_msgs::PoseStamped> transformed_plan;
	  //get the global plan in our frame
	  if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
		  ROS_WARN("Could not transform the global plan to the frame of the controller");
		  return false;
	  }
	  //now we'll prune the plan based on the position of the robot
	  if(prune_plan_)
		  prunePlan(global_pose, transformed_plan, global_plan_);

	  tf::Stamped<tf::Pose> drive_cmds;
	  drive_cmds.frame_id_ = robot_base_frame_;

	  tf::Stamped<tf::Pose> robot_vel;
	  odom_helper_.getRobotVel(robot_vel);

	  //if the global plan passed in is empty... we won't do anything
	  if(transformed_plan.empty())
		  return false;

	  tf::Stamped<tf::Pose> goal_point;
	  tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
	  //tf::Stamped<tf::Pose> goal_point;
	  //we assume the global goal is the last point in the global plan
	  double goal_x = goal_point.getOrigin().getX();
	  double goal_y = goal_point.getOrigin().getY();
	  double yaw = tf::getYaw(goal_point.getRotation());

	  double goal_th = yaw;

	  tf::Stamped<tf::Pose> start_pose;
	  tf::poseStampedMsgToTF(transformed_plan.front(), start_pose);
	  p_start.x = start_pose.getOrigin().getX();
	  p_start.y = start_pose.getOrigin().getY();
	  p_start.z = tf::getYaw(start_pose.getRotation());

	  goal_distance = getGoalPositionDistance(start_pose, goal_x, goal_y);
	  double start_distance = getGoalPositionDistance(global_pose, p_start.x, p_start.y);

	  p_end.x = goal_x;
	  p_end.y = goal_y;
	  p_end.z = goal_th;
	  control_style = "auto";

	  //计算路径直线与全局地图正方向间夹角
	  path_rot = cal_path_rotation(p_start,p_end);
	  path_rot = normalize_angle_me(path_rot);

	  //计算终点与当前点夹角
	  p_cur.x = global_pose.getOrigin().getX();
	  p_cur.y = global_pose.getOrigin().getY();
	  p_cur.z = tf::getYaw(global_pose.getRotation());

	  end_rot = cal_path_rotation(p_cur,p_end);
	  end_rot = normalize_angle_me(end_rot);

//	  std_msgs::String ops;
//	  ops.data = control_style;
//	  control_style_pub.publish(ops);

	  //compute what trajectory to drive along
	  //copy over the odometry information
	  nav_msgs::Odometry base_odom;
	  odom_helper_.getOdom(base_odom);

	  cmd_vel.linear.x = base_odom.twist.twist.linear.x;
	  cmd_vel.angular.z = base_odom.twist.twist.angular.z;
	  std_msgs::String dockstate;
	  dockstate.data = "next_goal:" + _next_goal.poseid_end + ";_carinfo.ChargeState:" + std::to_string(_batteryInfo.BC_Status);
	  dockstop_pub.publish(dockstate);

	  if(_batteryInfo.BC_Status == 1 && _next_goal.poseid_end == _DockPile){
		  //_emergencyStop = false;
		  cmd_vel.linear.x = 0.0;
		  cmd_vel.linear.y = 0.0;
		  cmd_vel.angular.z = 0.0;
		  rotating_to_goal_ = false;
		  xy_tolerance_latch_ = false;
		  reached_goal_ = true;

		  th_pid.Ierror =0;
		  th_pid.Perror =0;
		  th_pid.Poutput =0;
		  th_pid.PrevErr =0;
		  dis_pid.Ierror =0;
		  dis_pid.Perror =0;
		  dis_pid.Poutput =0;
		  dis_pid.PrevErr =0;

		  p_end.x = goal_x;
		  p_end.y = goal_y;
		  p_end.z = goal_th;

		  goal_distance = 0.0;

		  std_msgs::String pose_at;
		  pose_at.data = "Dockdone";
		  attitude_pub.publish(pose_at);
		  _First_back_done = false;
		  return true;
	  }
	  ROS_ERROR_DELAYED_THROTTLE(2.0,"_inspectInfo.Bepassinground = %u;_equipmentInfo.ObstacleWarning = %d;_First_back_done = %u", _Bepassinground, _equipmentInfo.ObstacleWarning,_First_back_done);

	  if((_Bepassinground == true && _equipmentInfo.ObstacleWarning > 0 && _First_back_done == false))
	  {
//	 	  if(getGoalPositionDistance(global_pose, goal_x, goal_y) < xy_goal_tolerance_ || start_distance > goal_distance)
//	 	  {
//			  //_emergencyStop = false;
//			  cmd_vel.linear.x = 0.0;
//			  cmd_vel.linear.y = 0.0;
//			  cmd_vel.angular.z = 0.0;
//			  rotating_to_goal_ = false;
//			  xy_tolerance_latch_ = false;
//			  reached_goal_ = true;
//
//			  th_pid.Ierror =0;
//			  th_pid.Perror =0;
//			  th_pid.Poutput =0;
//			  th_pid.PrevErr =0;
//			  dis_pid.Ierror =0;
//			  dis_pid.Perror =0;
//			  dis_pid.Poutput =0;
//			  dis_pid.PrevErr =0;
//
//			  p_end.x = goal_x;
//			  p_end.y = goal_y;
//			  p_end.z = goal_th;
//
//			  goal_distance = 0.0;
//			  std_msgs::String pose_at;
//			  pose_at.data = "taskdone back -1";
//			  if(_task_point_id == _DockPile){
//				  _First_back_done = false;
//			  }
//	 	  }

 		  cmd_vel.linear.x = escape_vel_;
 		  //cmd_vel.angular.z = 0.0;
 		  //_emergencyStop = false;
 		  geometry_msgs::Point cur_pos;
 		  cur_pos.x = global_pose.getOrigin().getX();
 		  cur_pos.y = global_pose.getOrigin().getY();

 		  double error_th = 0.0;
 		  double dis_pid_tmp = 0.0;
 		  double th_pid_tmp = 0.0;
 		  th_pid.Perror = (th_pid.Perror);
 		  dis_pid.Perror = (dis_pid.Perror);

 		  dis_pid_tmp = Ka_move_*CacPid(dis_pid);
 		  th_pid_tmp = Kb_move_*CacPid(th_pid);
 		  error_th = dis_pid_tmp + th_pid_tmp;
 		  //ROS_INFO("dis_pid_tmp = %.3f \n,th_pid_tmp=%.3f \n,error_th=%.3f",dis_pid_tmp,th_pid_tmp,error_th);
 		  cmd_vel.angular.z = error_th;

 		  if(cmd_vel.angular.z > max_vel_th_/4)
 		  {
 			  cmd_vel.angular.z = max_vel_th_/4;
 		  }
 		  if(cmd_vel.angular.z < min_vel_th_/4)
 		  {
 			  cmd_vel.angular.z = min_vel_th_/4;
 		  }

 		  //ROS_INFO("cmd_vel.angular.z= %.4f",cmd_vel.angular.z);
 		  th_pid.Perror = getGoalOrientationAngleDifference(global_pose, end_rot) + M_PI;
 		  th_pid.Perror = normalize_angle_me(th_pid.Perror);
 		  dis_pid.Perror = cal_distance(cur_pos, p_start, p_end);
		  //cmd_vel.angular.z = 0.0;
 		  std_msgs::String pose_at;
 		  pose_at.data = "backforward _Bepassinground";
 		  attitude_pub.publish(pose_at);
 		  return true;
	  }

	  ROS_ERROR_DELAYED_THROTTLE(2.0,"_task_arrived_id = %s;_task_point_id = %s",_task_arrived_id.c_str(),_task_point_id.c_str());
	  if((_task_arrived_id == _DockPoint && _task_point_id == _DockPile )/*||(_task_arrived_id == "ku" && _task_point_id == "cd")*/)
	  {
		  ROS_ERROR("cd to cz backward _task_arrived_id = %s----_task_point_id=%s",_task_arrived_id.c_str(),_task_point_id.c_str());
	 	  if(getGoalPositionDistance(global_pose, goal_x, goal_y) > xy_goal_tolerance_ && start_distance < goal_distance)
	 	  {
	 		  double angle_cz = getGoalOrientationAngleDifference(global_pose, goal_th);
			  if (fabs(angle_cz) > yaw_goal_tolerance_*2 /*&& cmd_vel.linear.x < 0.01*/)
			  {
			  	//set the velocity command to zero
			  	//if(_end_point_id == _task_point_id)
//			  	{
//                                ROS_INFO("11");
                                cmd_vel.linear.x = 0.0;
//				  cmd_vel.angular.z = sign(normalize_angle_me(angle_cz)) * min_in_place_vel_th_;
				  std_msgs::String pose_at;
				  pose_at.data = "turn 0";
				  attitude_pub.publish(pose_at);

				  static float angle = 0.0;
				  static int count = 0;
				  static ros::Time start_time; //= ros::Time::now();
				  static ros::Time end_time; //= ros::Time::now();
				  static bool stop_state = false;
				  if(count == 0){
					  start_time = ros::Time::now();
					  count++;
				  }else if(count == 1){
					  end_time = ros::Time::now();
					  count++;
				  }else{
					  start_time = end_time;
					  end_time = ros::Time::now();
				  }

				  if(fabs(angle) > delay_factor_/2.0){
					  static ros::Time stop_time_start; //= ros::Time::now();
					  static ros::Time stop_time_end; //= ros::Time::now();
					  static double stop_time = delay_time_;
					  static int count_delay = 0;
					  if(count_delay == 0){
						  stop_time_start = ros::Time::now();
						  count_delay++;
					  }else if(count_delay == 1){
						  stop_time_end = ros::Time::now();
						  count_delay++;
					  }else{
						  stop_time_start = stop_time_end;
						  stop_time_end = ros::Time::now();
					  }
					  if(stop_time >= 0.0){
						  stop_time -= fabs(stop_time_end.toSec() - stop_time_start.toSec());
						  cmd_vel.angular.z = 0.0;
						  if(stop_state == false){
							  stop_state = true;
							  std_msgs::Bool stop_state_s;
							  stop_state_s.data = stop_state;
								//power_stop_pub.publish(stop_state_s);
						  }
					  }else{
						  angle = 0;
						  stop_time = delay_time_;
						  stop_state = false;
					  }
				  }else{
					  cmd_vel.angular.z = sign(normalize_angle_me(angle_cz)) * min_in_place_vel_th_;//sign(normalize_angle_me(getGoalOrientationAngleDifference(global_pose, path_rot)))*min_in_place_vel_th_;
					  angle += fabs(cmd_vel.angular.z * (end_time.toSec() - start_time.toSec()));
				  }
				  return true;
	  		  }

	 		  cmd_vel.linear.x = escape_vel_;
	 		  geometry_msgs::Point cur_pos;
	 		  cur_pos.x = global_pose.getOrigin().getX();
	 		  cur_pos.y = global_pose.getOrigin().getY();

	 		  double error_th = 0.0;
	 		  double dis_pid_tmp = 0.0;
	 		  double th_pid_tmp = 0.0;
	 		  th_pid.Perror = (th_pid.Perror);
	 		  dis_pid.Perror = (dis_pid.Perror);

	 		  dis_pid_tmp = Ka_move_*CacPid(dis_pid);
	 		  th_pid_tmp = Kb_move_*CacPid(th_pid);
	 		  error_th = dis_pid_tmp + th_pid_tmp;
	 		  //ROS_INFO("dis_pid_tmp = %.3f \n,th_pid_tmp=%.3f \n,error_th=%.3f",dis_pid_tmp,th_pid_tmp,error_th);
	 		  cmd_vel.angular.z = error_th;

	 		  if(cmd_vel.angular.z > max_vel_th_/4)
	 		  {
	 			  cmd_vel.angular.z = max_vel_th_/4;
	 		  }
	 		  if(cmd_vel.angular.z < min_vel_th_/4)
	 		  {
	 			  cmd_vel.angular.z = min_vel_th_/4;
	 		  }
	 		  //ROS_INFO("cmd_vel.angular.z= %.4f",cmd_vel.angular.z);
	 		  th_pid.Perror = getGoalOrientationAngleDifference( global_pose, end_rot) + M_PI;
	 		  th_pid.Perror = normalize_angle_me(th_pid.Perror);
	 		  dis_pid.Perror = cal_distance( cur_pos, p_start, p_end);
			  //cmd_vel.angular.z = 0.0;
	 		  std_msgs::String pose_at;
	 		  pose_at.data = "backforward";
	 		  attitude_pub.publish(pose_at);

			  return true;
	 	  }

		  cmd_vel.linear.x = 0.0;
		  cmd_vel.linear.y = 0.0;
		  cmd_vel.angular.z = 0.0;
		  rotating_to_goal_ = false;
		  xy_tolerance_latch_ = false;
		  reached_goal_ = true;

		  th_pid.Ierror =0;
		  th_pid.Perror =0;
		  th_pid.Poutput =0;
		  th_pid.PrevErr =0;
		  dis_pid.Ierror =0;
		  dis_pid.Perror =0;
		  dis_pid.Poutput =0;
		  dis_pid.PrevErr =0;

		  p_end.x = goal_x;
		  p_end.y = goal_y;
		  p_end.z = goal_th;

		  goal_distance = 0.0;

		  std_msgs::String pose_at;
		  pose_at.data = "taskdone pile";
		  attitude_pub.publish(pose_at);
		  if(_task_arrived_id == _DockPoint){
			  _First_back_done = false;
		  }
 		  return true;
	  }

	  //check to see if we've reached the goal position
	  if (/*xy_tolerance_latch_ ||*/ getGoalPositionDistance(global_pose, goal_x, goal_y) < xy_goal_tolerance_ || (start_distance >= goal_distance)) {
		  //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
		  //just rotate in place
		  //      if (latch_xy_goal_tolerance_) {
		  //        xy_tolerance_latch_ = true;
		  //      }
		  //ROS_INFO("0");
		  double angle_in = getGoalOrientationAngleDifference(global_pose, goal_th);
		  //check to see if the goal orientation has been reached
		  if (fabs(angle_in) > yaw_goal_tolerance_)
		  {
			  //set the velocity command to zero
			  if(_end_point_id == _DockPoint/*|| _end_point_id == "ku"  */||_end_point_id == _stop_point_id)
			  {
				  ROS_INFO("cd turn action _DockPoint = %s,_end_point_id=%s",_DockPoint.c_str(),_end_point_id.c_str());
				  cmd_vel.linear.x = 0.0;
				  cmd_vel.angular.z = sign(normalize_angle_me(angle_in)) * min_in_place_vel_th_;
				  std_msgs::String pose_at;
				  pose_at.data = "turn 1";
				  attitude_pub.publish(pose_at);

//				  static float angle = 0.0;
//				  static int count = 0;
//				  static ros::Time start_time; //= ros::Time::now();
//				  static ros::Time end_time; //= ros::Time::now();
//				  static bool stop_state = false;
//				  if(count == 0){
//					  start_time = ros::Time::now();
//					  count++;
//				  }else if(count == 1){
//					  end_time = ros::Time::now();
//					  count++;
//				  }else{
//					  start_time = end_time;
//					  end_time = ros::Time::now();
//				  }
//				  if(fabs(angle) > delay_factor_/2.0){
//					  static ros::Time stop_time_start; //= ros::Time::now();
//					  static ros::Time stop_time_end; //= ros::Time::now();
//					  static double stop_time = delay_time_;
//					  static int count_delay = 0;
//					  if(count_delay == 0){
//						  stop_time_start = ros::Time::now();
//						  count_delay++;
//					  }else if(count_delay == 1){
//						  stop_time_end = ros::Time::now();
//						  count_delay++;
//					  }else{
//						  stop_time_start = stop_time_end;
//						  stop_time_end = ros::Time::now();
//					  }
//					  if(stop_time >= 0.0){
//						  stop_time -= fabs(stop_time_end.toSec() - stop_time_start.toSec());
//						  cmd_vel.angular.z = 0.0;
//						  if(stop_state == false){
//							  stop_state = true;
//							  std_msgs::Bool stop_state_s;
//							  stop_state_s.data = stop_state;
//								//power_stop_pub.publish(stop_state_s);
//						  }
//					  }else{
//						  angle = 0;
//						  stop_time = delay_time_;
//						  stop_state = false;
//					  }
//				  }else{
//					  cmd_vel.angular.z = sign(normalize_angle_me(angle_in))* min_in_place_vel_th_;//sign(normalize_angle_me(getGoalOrientationAngleDifference(global_pose, path_rot)))*min_in_place_vel_th_;
//					  angle += fabs(cmd_vel.angular.z * (end_time.toSec() - start_time.toSec()));
//				  }
				  return true;
			  }
		  }

		  cmd_vel.linear.x = 0.0;
		  cmd_vel.linear.y = 0.0;
		  cmd_vel.angular.z = 0.0;
		  rotating_to_goal_ = false;
		  xy_tolerance_latch_ = false;
		  reached_goal_ = true;

		  th_pid.Ierror=0;
		  th_pid.Perror=0;
		  th_pid.Poutput=0;
		  th_pid.PrevErr=0;
		  dis_pid.Ierror=0;
		  dis_pid.Perror=0;
		  dis_pid.Poutput=0;
		  dis_pid.PrevErr=0;

		  p_end.x = goal_x;
		  p_end.y = goal_y;
		  p_end.z = goal_th;

		  //path_rot = tf::getYaw(global_pose.getRotation());
		  goal_distance = 0.0;
		  //we need to call the next two lines to make sure that the trajectory
		  //planner updates its path distance and goal distance grids
		  //map_viz_.publishCostCloud(costmap_);
		  //nav_msgs::Odometry base_odom;
		  //odom_helper_.getOdom(base_odom);
		  std_msgs::String pose_at;
		  pose_at.data = "taskdone 1";
		  attitude_pub.publish(pose_at);
		  return true;
	  }

	  //copy over the odometry information
	  //base_odom.twist.twist.linear.x
      if(getGoalPositionDistance(global_pose, goal_x, goal_y) <= slowdown_goal_tolerance_ && start_distance <= (goal_distance -xy_goal_tolerance_)){
    	  //we need to call the next two lines to make sure that the trajectory
    	  //planner updates its path distance and goal distance grids
    	  tc_->updatePlan(transformed_plan);
    	  //if(path.cost_ < 0)
    	  //{
    	  //	path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
    	  //}
    	  //map_viz_.publishCostCloud(costmap_);
    	  //copy over the odometry information
//		  nav_msgs::Odometry base_odom;
//		  odom_helper_.getOdom(base_odom);
    	  //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
    	  if (getGoalPositionDistance(global_pose, goal_x, goal_y) > xy_goal_tolerance_ && start_distance <= (goal_distance - xy_goal_tolerance_))
    	  {
			  double cut_rot = fabs(normalize_angle_me(path_rot - tf::getYaw(global_pose.getRotation()))) ;
			  if(cut_rot > (M_PI/180) && debug_pid_ == false)
			  {
				  if(_start_point_id == _DockPile && _end_point_id == _DockPoint)
				  {
					  //ROS_ERROR_DELAYED_THROTTLE(2,"non turn in cz or cd in slow area");
					  ROS_ERROR_DELAYED_THROTTLE(2.0,"slow non turn in cd or ku in slow area; _start_point_id = %s; _end_point_id = %s",_start_point_id.c_str(),_end_point_id.c_str());
				  }
				  else
				  {
					if(cmd_vel.linear.x < 0.03)
					{
						cmd_vel.linear.x = 0.0;
						//cmd_vel.angular.z = sign(normalize_angle_me(getGoalOrientationAngleDifference(global_pose, end_rot)))*min_out_place_vel_th_;
						std_msgs::String pose_at;
						pose_at.data = "turn 2";
						attitude_pub.publish(pose_at);

//						static float angle = 0.0;
//						static int count = 0;
//						static ros::Time start_time; //= ros::Time::now();
//						static ros::Time end_time; //= ros::Time::now();
//						static bool stop_state = false;
//						if(count == 0){
//							start_time = ros::Time::now();
//							count++;
//						}else if(count == 1){
//							end_time = ros::Time::now();
//							count++;
//						}else{
//							start_time = end_time;
//							end_time = ros::Time::now();
//						}
//						if(fabs(angle) > delay_factor_){
//							static ros::Time stop_time_start; //= ros::Time::now();
//							static ros::Time stop_time_end; //= ros::Time::now();
//							static double stop_time = delay_time_;
//							static int count_delay = 0;
//							if(count_delay == 0){
//								stop_time_start = ros::Time::now();
//								count_delay++;
//							}else if(count_delay == 1){
//								stop_time_end = ros::Time::now();
//								count_delay++;
//							}else{
//								stop_time_start = stop_time_end;
//								stop_time_end = ros::Time::now();
//							}
//							if(stop_time >= 0.0){
//								stop_time -= fabs(stop_time_end.toSec() - stop_time_start.toSec());
//								cmd_vel.angular.z = 0.0;
//								if(stop_state == false){
//									stop_state = true;
//									std_msgs::Bool stop_state_s;
//									stop_state_s.data = stop_state;
//									//power_stop_pub.publish(stop_state_s);
//								}
//							}else{
//								angle = 0;
//								stop_time = delay_time_;
//								stop_state = false;
//							}
//						}else{
							cmd_vel.angular.z = sign(normalize_angle_me(getGoalOrientationAngleDifference(global_pose, path_rot)))*min_in_place_vel_th_;
//							angle += fabs(cmd_vel.angular.z * (end_time.toSec() - start_time.toSec()));
//						}
						return true;
					}
				  }
			  }

			  if( min_vel_x < cmd_vel.linear.x ){
				  cmd_vel.linear.x = std::max( min_vel_x, cmd_vel.linear.x - acc_lim_x_*3);//#0.02
			  }else{
					cmd_vel.linear.x = min_vel_x; //std::max( min_vel_x, cmd_vel.linear.x);
			  }

			  geometry_msgs::Point cur_pos;
			  cur_pos.x = global_pose.getOrigin().getX();
			  cur_pos.y = global_pose.getOrigin().getY();

			  double error_th = 0.0;
			  double dis_pid_tmp = 0.0;
			  double th_pid_tmp = 0.0;
			  th_pid.Perror = (th_pid.Perror);
			  dis_pid.Perror = (dis_pid.Perror);

			  dis_pid_tmp = Ka_move_*CacPid(dis_pid);
			  th_pid_tmp = Kb_move_*CacPid(th_pid);
			  error_th = dis_pid_tmp + th_pid_tmp;

			  //ROS_INFO("dis_pid_tmp = %.3f \n,th_pid_tmp=%.3f \n,error_th=%.3f",dis_pid_tmp,th_pid_tmp,error_th);
			  cmd_vel.angular.z = error_th;

			  if(_start_point_id == _DockPile && _end_point_id == _DockPoint){
					cmd_vel.angular.z = 0.0;
					std_msgs::String pose_at;
					pose_at.data = "forward_no_rectify";
					attitude_pub.publish(pose_at);
			  }

			  if(cmd_vel.angular.z > max_vel_th_){
				  cmd_vel.angular.z = max_vel_th_;
			  }
			  if(cmd_vel.angular.z < min_vel_th_)
			  {
				  cmd_vel.angular.z = min_vel_th_;
			  }
			  //cmd_vel.angular.z = 0.0;
			  th_pid.Perror = getGoalOrientationAngleDifference(global_pose, path_rot);
			  th_pid.Perror = normalize_angle_me(th_pid.Perror);
			  dis_pid.Perror = cal_distance(cur_pos, p_start, p_end);
    	  }

		  //publish an empty plan because we've reached our goal position
		  publishPlan(transformed_plan, g_plan_pub_);
		  publishPlan(transformed_plan, l_plan_pub_);
		  std_msgs::String pose_at;
		  pose_at.data = "move 2";
		  attitude_pub.publish(pose_at);

		  if(_Bepassinground == true && _equipmentInfo.ObstacleWarning == 0)
		  {
			  _First_back_done = true;
			  ROS_ERROR_DELAYED_THROTTLE(2.0,"turn around road go back first slow area");
		  }
    	  return true;
	  }

	  tc_->updatePlan(transformed_plan);
	  //compute what trajectory to drive along
	  //copy over the odometry information
	  //nav_msgs::Odometry base_odom;
	  //odom_helper_.getOdom(base_odom);

	  ROS_DEBUG_NAMED("trajectory_planner_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
				  cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

	  geometry_msgs::Point cur_pos;
	  cur_pos.x = global_pose.getOrigin().getX();
	  cur_pos.y = global_pose.getOrigin().getY();
	  //publish information to the visualizer
	  publishPlan(transformed_plan, g_plan_pub_);
	  publishPlan(transformed_plan, l_plan_pub_);

	  double cut_rot = fabs(normalize_angle_me(path_rot - tf::getYaw(global_pose.getRotation())));

	  if(cut_rot > (M_PI/60) && debug_pid_ == false/* && _next_goal.curvity == 0*/)
	  {
		  //ROS_WARN("1111111111111 cut_rot = %.3f",cut_rot);
		  if(_start_point_id == _DockPile && _end_point_id == _DockPoint)
		  {
			  ROS_ERROR_DELAYED_THROTTLE(2.0, "normal non turn in cd or ku in normal area; _start_point_id = %s; _end_point_id = %s",_start_point_id.c_str(),_end_point_id.c_str());
		  }else{
			  if(cmd_vel.linear.x < 0.03)
			  {
				  cmd_vel.linear.x = 0.0;
				  //cmd_vel.angular.z = sign(normalize_angle_me(getGoalOrientationAngleDifference(global_pose, path_rot)))*min_out_place_vel_th_;
				  std_msgs::String pose_at;
				  pose_at.data = "turn 3";
				  attitude_pub.publish(pose_at);

//					static float angle = 0.0;
//					static int count = 0;
//					static ros::Time start_time; //= ros::Time::now();
//					static ros::Time end_time; //= ros::Time::now();
//					static bool stop_state = false;
//					if(count == 0){
//						start_time = ros::Time::now();
//						count++;
//					}else if(count == 1){
//						end_time = ros::Time::now();
//						count++;
//					}else{
//						start_time = end_time;
//						end_time = ros::Time::now();
//					}
//					if(fabs(angle) > delay_factor_){
//						static ros::Time stop_time_start; //= ros::Time::now();
//						static ros::Time stop_time_end; //= ros::Time::now();
//						static double stop_time = delay_time_;
//						static int count_delay = 0;
//						if(count_delay == 0){
//							stop_time_start = ros::Time::now();
//							count_delay++;
//						}else if(count_delay == 1){
//							stop_time_end = ros::Time::now();
//							count_delay++;
//						}else{
//							stop_time_start = stop_time_end;
//							stop_time_end = ros::Time::now();
//						}
//						if(stop_time >= 0.0){
//							stop_time -= fabs(stop_time_end.toSec() - stop_time_start.toSec());
//							cmd_vel.angular.z = 0.0;
//							if(stop_state == false){
//								stop_state = true;
//								std_msgs::Bool stop_state_s;
//								stop_state_s.data = stop_state;
//								//power_stop_pub.publish(stop_state_s);
//							}
//						}else{
//							angle = 0;
//							stop_time = delay_time_;
//							stop_state = false;
//						}
//					}else{
						cmd_vel.angular.z = sign(normalize_angle_me(getGoalOrientationAngleDifference(global_pose, path_rot)))*min_in_place_vel_th_;
//						angle += fabs(cmd_vel.angular.z * (end_time.toSec() - start_time.toSec()));
//					}
					return true;
				}
			}
		}

		if(max_vel_x < cmd_vel.linear.x)
		{
			cmd_vel.linear.x = std::max(max_vel_x, cmd_vel.linear.x - acc_lim_x_*3);//#0.02
		}
		else
		{
			cmd_vel.linear.x = std::min(max_vel_x, cmd_vel.linear.x + acc_lim_x_);
		}

		//计算路径直线与全局地图正方向间夹角
		p_end.x = goal_x;
		p_end.y = goal_y;
		p_end.z = goal_th;

		double error_th = 0.0;
		double dis_pid_tmp = 0.0;
		double th_pid_tmp = 0.0;

		th_pid.Perror = (th_pid.Perror);
		dis_pid.Perror = (dis_pid.Perror);

		dis_pid_tmp = Ka_move_*CacPid(dis_pid);
		th_pid_tmp = Kb_move_*CacPid(th_pid);
		error_th = dis_pid_tmp + th_pid_tmp;

		cmd_vel.angular.z = error_th;

		if(_start_point_id == _DockPile && _end_point_id == _DockPoint)
		{
			cmd_vel.angular.z = 0.0;
		}

		if(cmd_vel.angular.z > max_vel_th_){
			cmd_vel.angular.z = max_vel_th_;
		}
		if(cmd_vel.angular.z < min_vel_th_)
		{
			cmd_vel.angular.z = min_vel_th_;
		}

		th_pid.Perror = getGoalOrientationAngleDifference(global_pose,path_rot);
		th_pid.Perror = normalize_angle_me(th_pid.Perror);
		dis_pid.Perror = cal_distance(cur_pos, p_start, p_end);
		publishPlan(transformed_plan, g_plan_pub_);
		publishPlan(transformed_plan, l_plan_pub_);
		std_msgs::String pose_at;
		pose_at.data = "move 3";
		attitude_pub.publish(pose_at);
		if(_Bepassinground == true && _equipmentInfo.ObstacleWarning == 0)
		{
			_First_back_done = true;
			ROS_ERROR_DELAYED_THROTTLE(2.0,"turn around road go back first");
		}
		return true;
  }

  bool TrajectoryPlannerROS::checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    tf::Stamped<tf::Pose> global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<geometry_msgs::PoseStamped> plan;
        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);
        plan.push_back(pose_msg);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      nav_msgs::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->checkTrajectory(global_pose.getOrigin().x(), global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()),
          base_odom.twist.twist.linear.x,
          base_odom.twist.twist.linear.y,
          base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);
    }
    ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return false;
  }


  double TrajectoryPlannerROS::scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    // Copy of checkTrajectory that returns a score instead of True / False
    tf::Stamped<tf::Pose> global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<geometry_msgs::PoseStamped> plan;
        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);
        plan.push_back(pose_msg);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      nav_msgs::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->scoreTrajectory(global_pose.getOrigin().x(), global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()),
          base_odom.twist.twist.linear.x,
          base_odom.twist.twist.linear.y,
          base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

    }
    ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return -1.0;
  }

  bool TrajectoryPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //return flag set in controller
    return reached_goal_; 
  }
};