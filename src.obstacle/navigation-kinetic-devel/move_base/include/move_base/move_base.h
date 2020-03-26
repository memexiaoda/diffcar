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
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>
#include <stdint.h>
#include <boost/unordered_map.hpp>
#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <carbot_msgs/Manualcmd.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <carbot_msgs/BatteryInfo.h>
#include <carbot_msgs/ControlInfo.h>
#include <carbot_msgs/DriverInfo.h>
#include <carbot_msgs/EquipmentInfo.h>
#include <carbot_msgs/InspectInfo.h>
#include <carbot_msgs/Carinfo.h>
#include <carbot_msgs/IDPose.h>
#include <carbot_msgs/IDPoseArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>

#include <boost/numeric/ublas/vector.hpp>  //生成一般的向量，其内置类型的值为0，在默认的情况下
#include <boost/numeric/ublas/io.hpp>
#include <exception>
//#include <boost/mpl/>
#include "websocket_server.h"
#include <carbot_msgs/LedMsg.h>
#include <carbot_msgs/Dock.h>
#include <carbot_msgs/Delayaction.h>
#include <carbot_msgs/Taskarrived.h>

#include <list>

#ifdef __cplusplus
extern "C" {
#endif

namespace move_base {
  enum MoveBaseState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  static const std::string scan_topic_ = "scan";

  typedef boost::function<void (void* p_msg_info)> CallbackType;
  typedef boost::function<void (msg_info* _msg_info)> CallbackType_Msg;

  class MoveBase;
  typedef void(MoveBase::*CallbackPtr)(void * p_msg_info);
  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  struct table_point_row{
	  std::string id;
	  std::string fname;
	  float lasermapX;
	  float lasermapY;
	  float lasermapTh;
	  float angle;
	  std::string direction;
	  int x;
	  int y;
	  std::string fun;
	  bool enable;
	  std::string robotid;

	  table_point_row(std::string id, std::string fname,
			  float lasermapX,float lasermapY,float lasermapTh,float angle,
			  std::string direction,int x,int y,std::string fun,bool enable,std::string robotid)
	  {
		  this->id = id;
		  this->fname = fname;
		  this->lasermapX = lasermapX;
		  this->lasermapY = lasermapY;
		  this->lasermapTh = lasermapTh;
		  this->angle = angle;
		  this->direction = direction;
		  this->x = x;
		  this->y = y;
		  this->fun = fun;
		  this->enable = enable;
		  this->robotid = robotid;
	  }

	  bool operator== (const table_point_row& t) const
	  {
		  return id==t.id && fname==t.fname &&
				  lasermapX==t.lasermapX &&  lasermapY==t.lasermapY &&  lasermapTh==t.lasermapTh &&
				  angle==t.angle && direction==t.direction &&
				  x==t.x && y==t.y &&
				  fun==t.fun && enable==t.enable && robotid==t.robotid;
	  }

	  size_t hash_value(const table_point_row& t)
	  {
	      size_t seed = 0;
	      boost::hash_combine(seed, boost::hash_value(t.id));
	      boost::hash_combine(seed, boost::hash_value(t.fname));
	      boost::hash_combine(seed, boost::hash_value(t.lasermapX));
	      boost::hash_combine(seed, boost::hash_value(t.lasermapY));
	      boost::hash_combine(seed, boost::hash_value(t.lasermapTh));
	      boost::hash_combine(seed, boost::hash_value(t.angle));
	      boost::hash_combine(seed, boost::hash_value(t.direction));
	      boost::hash_combine(seed, boost::hash_value(t.x));
	      boost::hash_combine(seed, boost::hash_value(t.y));
	      boost::hash_combine(seed, boost::hash_value(t.fun));
	      boost::hash_combine(seed, boost::hash_value(t.enable));
		  boost::hash_combine(seed, boost::hash_value(t.robotid));
	      return seed;
	  }
  };



  struct table_line_row{
	  std::string id;
	  std::string point1;
	  std::string point2;
	  float distance;
	  float turnangle;
	  std::string type;
	  bool enable;
	  std::string robotid;
	  int order;

	  table_line_row(std::string id, std::string point1,std::string point2,
			  float distance,float turnangle,
			  std::string type,bool enable,std::string robotid, int order)
	  {
		  this->id = id;
		  this->point1 = point1;
		  this->point2 = point2;
		  this->distance = distance;
		  this->turnangle = turnangle;
		  this->type = type;
		  this->enable = enable;
		  this->robotid = robotid;
		  this->order = order;
	  }

	  bool operator== (const table_line_row& t) const
	  {
		  return id==t.id && point1==t.point1 && point2==t.point2 &&
				  distance==t.distance &&  turnangle==t.turnangle &&
				  type==t.type && enable==t.enable && robotid==t.robotid && order==t.order;
	  }

	  size_t hash_value(const table_line_row& t)
	  {
	 	   size_t seed = 0;
	 	   boost::hash_combine(seed, boost::hash_value(t.id));
	 	   boost::hash_combine(seed, boost::hash_value(t.point1));
	 	   boost::hash_combine(seed, boost::hash_value(t.point2));
	 	   boost::hash_combine(seed, boost::hash_value(t.distance));
	 	   boost::hash_combine(seed, boost::hash_value(t.turnangle));
	 	   boost::hash_combine(seed, boost::hash_value(t.type));
	 	   boost::hash_combine(seed, boost::hash_value(t.enable));
	 	   boost::hash_combine(seed, boost::hash_value(t.robotid));
	 	   boost::hash_combine(seed, boost::hash_value(t.order));
	 	   return seed;
	  }
  };

  typedef boost::unordered_map<std::string, table_point_row> point_table_dic;
  typedef boost::unordered_map<std::string, table_line_row> line_table_dic;
  point_table_dic  Point_table,Point_table_origin;
  line_table_dic  Line_table,Line_table_origin;
  static int Nlength = 1000;
  static int Mlength = 1000;
#define _Nlength 1000
#define _Mlength 1000

  class map_value_finder
  {
  public:
         map_value_finder(const std::string &cmp_string):m_s_cmp_string(cmp_string){}
         bool operator ()(const boost::unordered_map<int, std::string>::value_type &pair)
         {
              return pair.second == m_s_cmp_string;
         }
  private:
          const std::string &m_s_cmp_string;
  };

  //template<class T>
  class map_value_finder_linelist
  {
  public:
	  map_value_finder_linelist(const std::string &cmp_string_point1,const std::string &cmp_string_point2)
  	  :m_s_cmp_string_point1(cmp_string_point1),m_s_cmp_string_point2(cmp_string_point2){}
	  bool operator ()(line_table_dic::value_type &pair)
	  {
		   return ((pair.second.point1 == m_s_cmp_string_point1)&& (pair.second.point2 == m_s_cmp_string_point2)) ;
	  }
	private:
	   const std::string &m_s_cmp_string_point1;
	   const std::string &m_s_cmp_string_point2;
  };

  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @param global_plan A reference to the global plan being used
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);
      /*
       * 重新对充电座
       */
      bool redockService(carbot_msgs::DockRequest & req,carbot_msgs::DockResponse & resp);
      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      //void laserCB(const sensor_msgs::LaserScan::ConstPtr& laser_scan_data);

      void planThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf::TransformListener& tf_;

      MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      unsigned int recovery_index_;

      tf::Stamped<tf::Pose> global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      double conservative_reset_dist_, clearing_radius_;
      //now
      ros::NodeHandle private_nh;
      ros::NodeHandle nh;

      geometry_msgs::Twist cmd_vel_cur;
      //actionlib::SimpleActionClient movebase
      int task_process_state_;
      std::string currentpoint_;
      std::string nextpoint_;
      std::string curgoalpoint_;
      std::string currentpoint_fname_;
      std::string nextpoint_fname_;
      std::string StopPoint_;
      std::string PathDone;
      int PowerAm;
      std::string DockPile;
      std::string DockPoint;
      std::string Carport;

      bool pose_init_;
      bool HasPointList;
      bool HasLineList;
      int webport_;
      ros::Publisher intial_pose_pub; //= rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 5)
      ros::Publisher cmd_pub;

      carbot_msgs::BatteryInfo _batteryinfo;
      carbot_msgs::ControlInfo _controlInfo;
      carbot_msgs::DriverInfo _driverInfo;
      carbot_msgs::EquipmentInfo _equipmentInfo;
      carbot_msgs::InspectInfo _InspectInfo;
      carbot_msgs::Carinfo _carinfo;

      bool Bepassinground;
      bool gohomestate;
      bool BC_Statu;
      bool BC_recharge;
      int Charge_count;
      std::string curtask_goal_id;

      msg_info m_msg_info;
      msg_info m_msg_info_state;

      double LObstacleStop_;
      double wheel_lenght;

      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
      ros::Publisher obstacle_pub;
      ros::Publisher task_posearray_pub;
      ros::Publisher manual_operation_pub;
      ros::Publisher led_command_pub;
      //ros::Publisher target_speed_pub;
      ros::Publisher control_style_pub;
      ros::Publisher Table_list_pub;

      std::string control_style;
      //rospy.Subscriber("/control_style", String, self.controlstyleCallback)

      double obstacle_dis;
      ros::Subscriber goal_sub_;
      ros::Subscriber arrived_sub_;

      ros::Subscriber InspectInfo_sub_;
      ros::Subscriber equipmentInfo_sub_;
      ros::Subscriber driverInfo_sub_;
      ros::Subscriber batteryinfo_sub_;
      ros::Subscriber current_task_sub;
      ros::Subscriber TaskProcessQ_sub;
      ros::Publisher All_info_pub;

      //ros::Subscriber laser_scan_sub;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      ros::ServiceServer delay_motion_srv_;
      ros::ServiceServer redock_srv;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      //set up the planner's thread
      bool runPlanner_;
      boost::mutex planner_mutex_;
      boost::condition_variable planner_cond_;
      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;
      boost::thread* websocketserver_thread_;
      boost::thread* command_filter_thread_;
      WebsocketServer websocket_server;

	  uint InspectInfo_seq_last;
      // Create a server endpoint
	  //server echo_server;
      inline void CarGo_process(double linear,double angular){
    	  gohomestate = false;
    	  carbot_msgs::Manualcmd manual_cmd;
		  manual_cmd.action_cmd = "forward";
		  manual_cmd.vel_value.linear.x = linear;
		  manual_cmd.vel_value.angular.z = angular;
		  manual_operation_pub.publish(manual_cmd);
		  control_style = "manual";
		  std_msgs::String style;
		  style.data = control_style;
		  control_style_pub.publish(style);
		  //gohomestate = false;
      }

      inline void CarTurnLeft_process(double linear,double angular){
    	  gohomestate = false;
    	  carbot_msgs::Manualcmd manual_cmd;
		  manual_cmd.action_cmd = "turnleft";
		  manual_cmd.vel_value.linear.x = linear;
		  manual_cmd.vel_value.angular.z = angular;
		  manual_operation_pub.publish(manual_cmd);
		  control_style = "manual";
		  std_msgs::String style;
		  style.data = control_style;
		  control_style_pub.publish(style);
		  //gohomestate = false;
		  //ros::Duration  rate;
		  //rate = ros::Duration(1.0);
		  //rate.sleep();
      }

      inline void CarTurnRight_process(double linear,double angular)
	  {
    	  gohomestate = false;
		  carbot_msgs::Manualcmd manual_cmd;
		  manual_cmd.action_cmd = "turnright";
		  manual_cmd.vel_value.linear.x = linear;
		  manual_cmd.vel_value.angular.z = angular;
		  manual_operation_pub.publish(manual_cmd);
		  control_style = "manual";
		  std_msgs::String style;
		  style.data = control_style;
		  control_style_pub.publish(style);
		  //gohomestate = false;
			//		  ros::Duration  rate;
			//		  rate = ros::Duration(1.0);
			//		  rate.sleep();
	  }

      inline void CarBack_process(double linear,double angular){
    	  gohomestate = false;
		  carbot_msgs::Manualcmd manual_cmd;
		  manual_cmd.action_cmd = "backward";
		  manual_cmd.vel_value.linear.x = linear;
		  manual_cmd.vel_value.angular.z = angular;
		  manual_operation_pub.publish(manual_cmd);
		  control_style = "manual";
		  std_msgs::String style;
		  style.data = control_style;
		  control_style_pub.publish(style);
		  //gohomestate = false;
			//		  ros::Duration  rate;
			//		  rate = ros::Duration(1.0);
			//		  rate.sleep();
	  }

      inline void CarStop_process(double linear,double angular){
    	  gohomestate = false;
		  carbot_msgs::Manualcmd manual_cmd;
		  manual_cmd.action_cmd = "stop";
		  manual_cmd.vel_value.linear.x = linear;
		  manual_cmd.vel_value.angular.z = angular;
		  control_style = "manual";
		  std_msgs::String style;
		  style.data = control_style;
		  control_style_pub.publish(style);
		  manual_operation_pub.publish(manual_cmd);
		  ros::Duration  rate;
		  rate = ros::Duration(1.0);
		  rate.sleep();;
	  }

      inline void CarStop_pause(double linear,double angular){
    	  gohomestate = false;
		  carbot_msgs::Manualcmd manual_cmd;
		  manual_cmd.action_cmd = "pause";
		  manual_cmd.vel_value.linear.x = linear;
		  manual_cmd.vel_value.angular.z = angular;
		  //manual_operation_pub.publish(manual_cmd);
		  control_style = "manual";
		  std_msgs::String style;
		  style.data = control_style;
		  control_style_pub.publish(style);
//		  gohomestate = false;
//		  ros::Duration  rate;
//		  rate = ros::Duration(1.0);
//		  rate.sleep();
	  }

      inline void CarStop_recover(double linear,double angular){
      		  carbot_msgs::Manualcmd manual_cmd;
      		  manual_cmd.action_cmd = "recover";
      		  manual_cmd.vel_value.linear.x = linear;
      		  manual_cmd.vel_value.angular.z = angular;
      		  //manual_operation_pub.publish(manual_cmd);
      		  control_style = "auto";
      		  std_msgs::String style;
      		  style.data = control_style;
      		  control_style_pub.publish(style);
      //		  ros::Duration  rate;
      //		  rate = ros::Duration(1.0);
      //		  rate.sleep();
	  }

	  struct MGraph
	  {
    	  float matrix[_Nlength][_Mlength];
		  int n;                             //顶点数
		  int e;                             //边数
		  void init()
		  {
			  //matrix = new float[Nlength][Mlength];

			  for (int i = 0; i < Nlength; i++)
			  {
				  for (int j = 0; j < Mlength; j++)
				  {
					  matrix[i][j] = 0.0f;
				  }
			  }
		  }
	  };

	  typedef boost::unordered_map<int,std::string> syncfname_pointList;
	  typedef boost::unordered_map<std::string, std::string> syncfname_preList;
	  //boost::find_iterator
	  //typedef boost::numeric::ublas::vector< std::pair<std::string, int>> syncfname_pointList;

	  void DijkstraPath(MGraph g, float dist[] , int path[], int v0)
	  {
		  bool visited[1000];
		  for (int i = 0; i < g.n; i++)
		  {
			  if (g.matrix[v0][i] > 0 && i != v0)
			  {
				  dist[i] = g.matrix[v0][i];
				  path[i] = v0;     //path记录最短路径上从v0到i的前一个顶点
			  }
			  else
			  {
				  dist[i] = INT_MAX;    //若i不与v0直接相邻，则权值置为无穷大
				  path[i] = -1;
			  }

			  visited[i] = false;
		  }

		  path[v0] = v0;
		  dist[v0] = 0;
		  visited[v0] = true;

		  for (int i = 1; i < g.n; i++) //循环扩展n-1次
		  {
			  float min = INT_MAX;
			  int u = 0;

			  for (int j = 0; j < g.n; j++) //寻找未被扩展的权值最小的顶点
			  {
				  if (visited[j] == false && dist[j] < min)
				  {
					  min = dist[j];
					  u = j;
				  }
			  }

			  visited[u] = true;

			  for (int k = 0; k < g.n; k++)   //更新dist数组的值和路径的值
			  {
				  if (visited[k] == false && g.matrix[u][k] > 0 && min + g.matrix[u][k] < dist[k])
				  {
					  dist[k] = min + g.matrix[u][k];
					  path[k] = u;
				  }
			  }
		  }
	  }

	  void CalcStopPoint(std::string param,carbot_msgs::IDPoseArray& task_path,bool& passingground)
	  {
		  ROS_ERROR("passingground = %u CalcStopPoint +++++++++++++++++++++", passingground);
		  point_table_dic::iterator point_iter;
		  line_table_dic::iterator line_iter;
		  Line_table = Line_table_origin;
		  Point_table = Point_table_origin;

		  if(curgoalpoint_ == "nopoint" || curgoalpoint_ == ""){
			  //curgoalpoint_ = currentpoint_;
			  ROS_ERROR("CalcStopPoint curgoalpoint_+++++++++++++++++++++", curgoalpoint_);
		  }

		  for (line_iter = Line_table.begin(); line_iter != Line_table.end(); ++line_iter)
		  {
			std::cout << "0000000000000line_id=" << line_iter->second.id<<"--enable="<< line_iter->second.enable<< std::endl;
		  }
    	  //task_poses_.poses.push_back();
		  //int i, j;
		  int s, t, w;      //表示存在一条边s->t,权值为w
		  MGraph g ;//= new MGraph;
		  g.init();
		  int v0;
		  syncfname_pointList _syncfname_pointList;
		  syncfname_preList  _syncfname_preList;
		  //syncfname_pointList::iterator syncfname_pointList_iter;

		  if(_InspectInfo.header.seq == InspectInfo_seq_last){
			  ROS_ERROR("CalcStopPoint _InspectInfo msg connect error:seq %d",_InspectInfo.header.seq);
			  return;
		  }else{
			  InspectInfo_seq_last = _InspectInfo.header.seq;
		  }

		  ROS_ERROR("11");
		  std::string cur_lineid = _InspectInfo.LineID;//_InspectInfo.StartPoint + "-" + _InspectInfo.EndPoint;
		  static bool origin_cur_lineid_enble;

		  if(_InspectInfo.MoveStateAction != "stop_at")
		  {
			  try{
				  //加入临时拓扑点
				  std::string id = "tmpoint";
				  std::string fname = "tmpoint";
				  float lasermapX = _InspectInfo.PosX;
				  float lasermapY = _InspectInfo.PosY;
				  float lasermapTh = _InspectInfo.Angle;
				  float angle = angle;
				  std::string direction;
				  if(fabs(_InspectInfo.Angle) <= 0.785){
					  direction = "a";
				  }
				  if(_InspectInfo.Angle > 0.785 && _InspectInfo.Angle <= 2.355){
					  direction = "b";
				  }
				  if(_InspectInfo.Angle > -2.355 && _InspectInfo.Angle <= -0.785){
					  direction = "c";
				  }
				  if(fabs(_InspectInfo.Angle) > 2.355){
					  direction = "d";
				  }
				  int x = 0;
				  int y = 0;
				  std::string fun = "";
				  bool enable = true;
				  std::string robotid = "3bot";
				  table_point_row row( id, fname, lasermapX, lasermapY, lasermapTh, angle,
															  direction, x, y, fun, enable, robotid);
				  Point_table.insert(point_table_dic::value_type(id,row));
			  }catch(std::exception& error){
				  ROS_ERROR("CarToStopPoint first error %s",error.what());
			  }
			  ROS_ERROR("33");
			  //加入临时拓扑线
			  std::string line_id;
			  std::string point1;
			  std::string point2;
			  float distance;
			  float turnangle;
			  std::string type;
			  bool enable;
			  std::string robotid;
			  int order;
			  std::string forwardline = _InspectInfo.LineID;
			  std::string backline = std::string(_InspectInfo.EndPoint + "-" + _InspectInfo.StartPoint);
			  line_table_dic::iterator line_iter1,line_iter2;
			  line_iter1 = Line_table.find(forwardline);
			  if(line_iter1 != Line_table.end())
			  {
				  if(Line_table.find(forwardline)->second.enable && passingground == false)
				  {
					  line_id = "tmpforward";
					  point1 = "tmpoint";
					  point2 = _InspectInfo.EndPoint;
					  float EndPointX = Point_table.find(_InspectInfo.EndPoint)->second.lasermapX;
					  float EndPointY = Point_table.find(_InspectInfo.EndPoint)->second.lasermapY;
					  distance = Line_table.find(_InspectInfo.LineID)->second.distance*2;//sqrt(pow((_InspectInfo.PosX - EndPointX),2) + pow((_InspectInfo.PosY - EndPointY),2));
					  turnangle = 0.0;
					  std::string type = "laser";
					  enable = true;
					  std::string robotid = "R01";
					  order = 1;
					  table_line_row row1(line_id, point1, point2, distance, turnangle, type, enable, robotid, order);
					  Line_table.insert(line_table_dic::value_type(line_id,row1));
				  }
			  }

			  ROS_ERROR("44");
			  line_id = "tmpbackward";
			  point1 = "tmpoint";
			  point2 = curgoalpoint_;
			  float StartPointX = Point_table.find(point2)->second.lasermapX;
			  float StartPointY = Point_table.find(point2)->second.lasermapY;
			  distance = sqrt(pow((_InspectInfo.PosX - StartPointX),2) + pow((_InspectInfo.PosY - StartPointY),2));
			  turnangle = 180.0;
			  type = "laser";
			  enable = true;
			  order = 1;
			  table_line_row row2(line_id, point1, point2, distance, turnangle, type, enable, robotid, order);
			  Line_table.insert(line_table_dic::value_type(line_id,row2));
			  ROS_ERROR("55");
			  //使当前拓扑线失效
			  line_iter = Line_table.find(cur_lineid);
			  if(line_iter != Line_table.end()){
				  origin_cur_lineid_enble = line_iter->second.enable;
			  }
			  ROS_ERROR("55-66-1");
			  if(origin_cur_lineid_enble == true){
				  line_iter->second.enable = false;
				  ROS_ERROR("55-66-2");
				  std::cout << "----------------------line_id=" << line_iter->second.id<< "--enable="<< line_iter->second.enable << "_InspectInfo.LineID"<< _InspectInfo.LineID << std::endl;
			  }
		  }

		  //计算当前点线表个数
		  int rowsp = 0;//lpoint.Rows.Count;
		  point_iter = Point_table.begin();
		  for(int i=0,rown=0; i < Point_table.size();  i++, ++point_iter)
		  {
			  std::string point1 = "";
			  point1 = point_iter->second.id;

			  if(point_iter->second.enable == true){
				  _syncfname_pointList.insert(syncfname_pointList::value_type(rown,point1));
				  std::cout << "point1=" << point1<<"--row="<< rown <<std::endl;
				  rowsp++;
				  rown++;
			  }
		  }
		  g.n = rowsp;
		  std::cout << "rowsp" << rowsp << std::endl;
		  ROS_ERROR("66");
		  int rowsl = 0;
		  for (line_iter = Line_table.begin(); line_iter != Line_table.end(); ++line_iter)
		  {
			  if(line_iter->second.enable == true){
				  std::cout << "line_id=" << line_iter->second.id<< "--rowsl="<< rowsl <<"--enable="<< line_iter->second.enable<< std::endl;
				  rowsl++;
			  }
		  }
		  std::cout << "rowsl" << rowsl << std::endl;
		  ROS_ERROR("77");
		  g.e = rowsl;
		  Nlength = g.n;
		  Mlength = g.e;

		  for (int i = 0; i < Nlength; i++)
		  {
			  for (int j = 0; j < Mlength; j++)
			  {
				  g.matrix[i][j] = 0.0f;
			  }
		  }
		  syncfname_pointList::iterator it;
		  ROS_ERROR("88");

		  for (line_iter = Line_table.begin(); line_iter != Line_table.end(); ++line_iter)
		  {
			  if(line_iter->second.enable == false)
			  {
				  std::cout << "line_id_false=" << line_iter->second.id << "--line_enable" << line_iter->second.enable << std::endl;
				  continue;
			  }

			  std::string point1 = "";
			  std::string point2 = "";
			  float wt = 0.0f;
			  wt = line_iter->second.distance;
			  point1 = line_iter->second.point1;
			  point2 = line_iter->second.point2;
			  std::cout << "point1:" << point1 << "--------point2:" << point2 << std::endl;
			  it = std::find_if(_syncfname_pointList.begin(), _syncfname_pointList.end(), map_value_finder(point1));
			  if(it != _syncfname_pointList.end())
			  {
				  s = it->first;
				  ROS_INFO("s=%d",s);
			  }

			  it = std::find_if(_syncfname_pointList.begin(), _syncfname_pointList.end(), map_value_finder(point2));

			  if(it != _syncfname_pointList.end())
			  {
				  t = it->first;
				  ROS_INFO("t=%d",t);
			  }

			  if (wt == 0) {
				  wt = 0.01f;
			  }

			  g.matrix[s][t] = wt;
			  ROS_INFO("wt=%f",wt);
		  }
		  ROS_ERROR("99");
		  if(_InspectInfo.MoveStateAction != "stop_at")
		  {
			  it = std::find_if(_syncfname_pointList.begin(), _syncfname_pointList.end(), map_value_finder("tmpoint"));
		  }
		  else{
			  it = std::find_if(_syncfname_pointList.begin(), _syncfname_pointList.end(), map_value_finder(curgoalpoint_));
		  }
		  ROS_ERROR("110");
		  if(it != _syncfname_pointList.end())
		  {
			  v0 = it->first;
			  ROS_INFO("v0=%d",v0);
		  }

		  float dist[1000] = {0.0f};
		  int path[1000] = {0};
		  ROS_ERROR("120");
		  DijkstraPath(g, dist, path, v0);
		  std::cout << "DijkstraPath" << std::endl;

		  for(int i=0; i < g.n; i++)
		  {
			  if (i!=v0 && path[i] > -1)
			  {
				  std::cout <<"i="<< i<<";v0="<<v0<<"--------------------"<< "path[i]="<<path[i]<< std::endl;
				  //syncfname_pointList_iter =  _syncfname_pointList.begin();
				  std::cout << _syncfname_pointList.find(i)->second << std::endl;
				  //syncfname_pointList_iter = _syncfname_pointList.begin();
				  std::cout << _syncfname_pointList.find(path[i])->second << std::endl;
				  _syncfname_preList.insert(syncfname_preList::value_type(_syncfname_pointList.find(i)->second,_syncfname_pointList.find(path[i])->second));
				  std::cout << " _syncfname_preList.end()->first:" << _syncfname_preList.begin()->first << " _syncfname_preList.end()->second:"<<_syncfname_preList.begin()->second << std::endl;
			  }
			  else
			  {
				  //std::cout <<"i:"<<i<<"v0:"<<v0<<"path["<<i<<"]="<<path[i]<< "---------- _syncfname_preList.end()->first:" << _syncfname_preList.begin()->first << " _syncfname_preList.end()->second:"<<_syncfname_preList.begin()->second << std::endl;
				  if (path[i] == -1)
				  {

				  }
			  }
		  }

		  ROS_ERROR("130");
		  std::string startpoint;
		  if(_InspectInfo.MoveStateAction != "stop_at")
		  {
			  startpoint = "tmpoint";//currentpoint_;//curgoalpoint_;
		  }else{
			  startpoint = curgoalpoint_;//curgoalpoint_;
		  }
		  ROS_ERROR("140");
		  std::string curpoint = param;
		  std::string prepoint = "";
		  int point_count = 0;
		  carbot_msgs::IDPose pose;
		  carbot_msgs::IDPoseArray task_path_desc;
		  line_table_dic::iterator line_it;

		  while (curpoint != startpoint)
		  {
			 syncfname_preList::iterator preList_iter = _syncfname_preList.begin();
			 preList_iter = _syncfname_preList.find(curpoint);
			 if (preList_iter != _syncfname_preList.end())
			 {
				 prepoint = curpoint;
				 curpoint = _syncfname_preList.find(curpoint)->second;
				 if(curpoint != startpoint){
					 pose.poseid_end =  prepoint;
					 pose.poseid_start = curpoint;
					 line_it = std::find_if(Line_table.begin(), Line_table.end(),map_value_finder_linelist(pose.poseid_start,pose.poseid_end));
					 if(line_it != Line_table.end())
					 {
						 pose.line_id = line_it->first;
						 if(pose.line_id == "tmpforward"){
							 pose.poseid_start = _InspectInfo.StartPoint;
						 }
						 if(pose.line_id == "tmpbackward"){
							 pose.poseid_start = _InspectInfo.EndPoint;
						 }
					     point_iter = Point_table.find(prepoint);
					     pose.passinground_task = passingground;//任务属性标记绕行性质
					     pose.curvity = line_it->second.turnangle;
						 pose.pose_corner = point_iter->second.fun;

						 pose.pose.position.x = point_iter->second.lasermapX;
						 pose.pose.position.y = point_iter->second.lasermapY;
						 pose.pose.orientation.w = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).w();
						 pose.pose.orientation.x = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).x();
						 pose.pose.orientation.y = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).y();
						 pose.pose.orientation.z = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).z();//new tf::Quaternion(0,0,Point_table.find(curpoint)->second.lasermapTh);

						 point_iter = Point_table.find(curpoint);
						 pose.pose_start.position.x = point_iter->second.lasermapX;
						 pose.pose_start.position.y = point_iter->second.lasermapY;
						 pose.pose_start.orientation.w = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).w();
						 pose.pose_start.orientation.x = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).x();
						 pose.pose_start.orientation.y = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).y();
						 pose.pose_start.orientation.z = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).z();

						 task_path_desc.idposearray.push_back(pose);
						 std::cout << "prepoint" << prepoint << "curpoint" << curpoint << std::endl;
					 }else{
						 ROS_ERROR("1_start_point=%s,end_point=%s",pose.poseid_start.c_str(),pose.poseid_end.c_str());
						 //line_iter = Line_table.begin();
						 //line_iter = Line_table.begin();
						 //line_iter = Line_table.find(cur_lineid);
						 //line_iter->second.enable = origin_cur_lineid_enble;
						 //std::cout << "9999999line_id=" << line_iter->second.id<<"--enable="<< line_iter->second.enable<< std::endl;
 						 return;
					}
				}
			 }
			 else
			 {
				 ROS_WARN("can't find _syncfname_preList.find(curpoint is %s) and currentpoint_ is %s and param is %s",curpoint.c_str(),currentpoint_.c_str(),param.c_str());
			 }
			 point_count++;
			 if(point_count>_syncfname_preList.size())
			 {
				 ROS_ERROR("point_count > _syncfname_preList.size()");
				 return;
			 }
		  }

		  ROS_ERROR("140");
		  pose.poseid_end = prepoint;
		  pose.poseid_start = curpoint;
		  line_it = std::find_if(Line_table.begin(),Line_table.end(),map_value_finder_linelist(pose.poseid_start,pose.poseid_end));

		  if(line_it != Line_table.end())
		  {
			  pose.line_id = line_it->first;
			  if(pose.line_id == "tmpforward"){
				  pose.poseid_start = _InspectInfo.StartPoint;
			  }
			  if(pose.line_id == "tmpbackward"){
				  pose.poseid_start = _InspectInfo.EndPoint;
			  }

			  point_iter = Point_table.find(prepoint);
			  pose.passinground_task = passingground;//任务属性标记绕行性质
			  pose.pose_corner = point_iter->second.fun;
			  pose.curvity = line_it->second.turnangle;
			  pose.pose.position.x = point_iter->second.lasermapX;
			  pose.pose.position.y = point_iter->second.lasermapY;
			  pose.pose.orientation.w = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).w();
			  pose.pose.orientation.x = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).x();
			  pose.pose.orientation.y = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).y();
			  pose.pose.orientation.z = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).z();//new tf::Quaternion(0,0,Point_table.find(curpoint)->second.lasermapTh);

			  point_iter = Point_table.find(curpoint);
			  pose.pose_start.position.x = point_iter->second.lasermapX;
			  pose.pose_start.position.y = point_iter->second.lasermapY;
			  pose.pose_start.orientation.w = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).w();
			  pose.pose_start.orientation.x = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).x();
			  pose.pose_start.orientation.y = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).y();
			  pose.pose_start.orientation.z = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).z();

			  task_path_desc.idposearray.push_back(pose);
			  std::cout << "prepoint:" << prepoint << "-curpoint:" << curpoint << std::endl;
			  point_count++;
		  }else{
			   ROS_ERROR("2_start_point=%s,end_point=%s",pose.poseid_start.c_str(),pose.poseid_end.c_str());
			   return;
		  }

		  ROS_ERROR("150");
		  carbot_msgs::IDPoseArray::_idposearray_type::iterator poseiter;
		  poseiter = --task_path_desc.idposearray.end();

		  while(poseiter != task_path_desc.idposearray.begin()){
			  task_path.idposearray.push_back(*poseiter);
			  std::cout<<"x="<< (*poseiter).pose.position.x<<"y="<<(*poseiter).pose.position.y<<"z"<<(*poseiter).pose.position.z << std::endl;
			  poseiter--;
		  }

		  task_path.idposearray.push_back(*poseiter);
		  std::cout<<"x="<< (*poseiter).pose.position.x << "y=" << (*poseiter).pose.position.y << "z" << (*poseiter).pose.position.z << std::endl;
		  task_path_desc.idposearray.clear();

		  point_iter = Point_table.find("tmpoint");
		  if(point_iter != Point_table.end()){
			  Point_table.erase(Point_table.find("tmpoint"));
		  }
		  line_iter =  Line_table.find("tmpforward");
		  if(line_iter != Line_table.end()){
			  Line_table.erase(Line_table.find("tmpforward"));
		  }
		  line_iter = Line_table.find("tmpbackward");
		  if(line_iter != Line_table.end()){
			  Line_table.erase(Line_table.find("tmpbackward"));
		  }
		  line_iter = Line_table.find(cur_lineid);
		  if(line_iter != Line_table.end()){
			  line_iter->second.enable = origin_cur_lineid_enble;
		  }
		  return;
	  }

      int Gostoppoint(std::string param,bool& passingground)
	  {
    	  ROS_ERROR("passingground = %u Gostoppoint 111111111111111111111", passingground);
    	  //std::string param_fname = Point_table.find(param)->second.fname;
    	  if (param == curgoalpoint_)
		  {
			  Json::Value new_item,new_item_1,new_item_2;
			  std::string returnStr;
			  new_item_1["ErrorCode"] = 0;
			  new_item_1["Resultdata"] = "";
			  Json::FastWriter writer;
			  std::string ResultStr = writer.write(new_item_1);
			  new_item_2["StopID"] = curgoalpoint_;
			  new_item_2["cor_x"] = 0.0;
			  new_item_2["cor_y"] = 0.0;
			  new_item_2["cor_angle"] = 0.0;
			  std::string ParamStr = writer.write(new_item_2);
			  new_item["Token"] = "";
			  new_item["Topic"] = "CarArrivedPoint";
			  boost::uuids::uuid uuid = boost::uuids::random_generator()();
			  std::string struuid = boost::lexical_cast<std::string>(uuid);
			  new_item["SessionId"] = struuid;
			  time_t t;  //秒时间
			  t = time(NULL); //获取目前秒时间
			  new_item["Time"] = (Json::Int64)t;
			  new_item["Param"] = ParamStr;
			  new_item["Result"] = ResultStr;
			  returnStr = writer.write(new_item);
			  ROS_ERROR("is already in goal");

			  echo_server.send(m_msg_info.client_hdl,returnStr, (m_msg_info.message)->get_opcode());
				  std::cout<< "task_arrived------ is already in goal" << "on_message called with hdl: " << (m_msg_info.client_hdl).lock().get()
						<< " and message: " << (m_msg_info.message)->get_payload()<< std::endl;

			  return 0;

		  }

		  carbot_msgs::IDPoseArray ptask_path;
		  ptask_path.header.stamp = ros::Time::now();
		  ptask_path.header.frame_id = "map";
		  ROS_ERROR("stop point param is %s",param.c_str());
		  CalcStopPoint(param, ptask_path,passingground);

		  if ( ptask_path.idposearray.size() > 0)
		  {
			  ROS_INFO("find path success");
		  }
		  else
		  {
			  ROS_ERROR("find path error");
			  return 1;
		  }
		  task_posearray_pub.publish(ptask_path);
		  return 0;
	  }


       typedef enum{
      	  CarGo = 1, //0
      	  CarTurnLeft,
      	  CarTurnRight,
      	  CarBack,
      	  CarStop,
      	  CarUpDataPoint,
      	  CarUpDataLine,
      	  CarToStopPoint,
      	  CarToTurnAngle,
      	  CarCharge,
      	  CarGoHome,
      	  CarTyreDiameter,
      	  CarLampControl,
      	  CarRadarDistance,
      	  CarMaxSpeed,
      	  CarCollisionAvoidance,
      	  CarClearPassDone,
      	  CarSuspendInspect,
      	  CarRecoverInspect,
		  CarEnablePointAndLine,
		  CarPowerAmount
      } CmdcIndex;

      typedef boost::unordered_map<std::string,CmdcIndex> Dic_Cmdc;
      Dic_Cmdc dic_cmdc;
      boost::mutex point_table_send_mutex_;
      boost::mutex line_table_send_mutex_;
      std::string MoveState;


      struct EnableCarStop{
    	  std::string ID;
    	  bool Enable;
      };

      typedef std::list<EnableCarStop> stoplist;

      struct RobotEnableRoute{
    	  std::string RobotID;
    	  stoplist EnableCarPointList;
    	  stoplist EnableCarLineList;
      };

      void command_parse(msg_info* p_msg_info/*,boost::thread& handle*/){
    	  Json::Reader reader;
    	  Json::Value root;
    	  std::string msg_data;
    	  msg_data = (p_msg_info->message)->get_payload();
    	  try {
			  if (reader.parse(msg_data, root))  // reader将Json字符串解析到root，root将包含Json里所有子元素
			  {
				  ROS_WARN("command_parse in 1111111");
				  // 转换为字符串（带格式）
				  std::string out = root.toStyledString();
				  std::string Token = root["Token"].asString();
				  std::string Topic = root["Topic"].asString();  // 访问节点，upload_id = "UP000000"
				  std::string SessionId = root["SessionId"].asString();
				  unsigned long long Time = root["Time"].asUInt64();
				  std::string Result = root["Result"].asString();
				  Json::Value Paramroot;

				  //msg_info send_msgs;
				  Json::Value returnObj;   // 构建对象
				  Json::Value ResultObj;
				  Json::Value new_item,new_item_1,new_item_2;//Token_item, Topic_item, SessionId_item, Time_item,Param_item,ErrorCode_item,Resultdata_item;
				  std::string returnStr;
				  CmdcIndex cmdcIndex;

//				  try{
//					  cmdcIndex = dic_cmdc.find(Topic)->second;
//				  }catch(std::exception& error){
//					  ROS_ERROR("cmdcIndex find error %s",error.what());
//				  }
				  /*
				   * CarGo = 1, //0
				  	 CarTurnLeft,
					 CarTurnRight,
					 CarBack,
					 CarStop,
					 CarUpDataPoint,
					 CarUpDataLine,
					 CarToStopPoint,
					 CarToTurnAngle,
					 CarCharge,
					 CarGoHome,
					 CarTyreDiameter,
					 CarLampControl,
					 CarRadarDistance,
					 CarMaxSpeed,
					 CarCollisionAvoidance,
					 CarClearPassDone,
					 CarSuspendInspect,
					 CarRecoverInspect
				   */
				  if(Topic == "CarGo"){
					  cmdcIndex = MoveBase::CarGo;
				  }else if (Topic == "CarTurnLeft"){
					  cmdcIndex = MoveBase::CarTurnLeft;
				  }else if (Topic == "CarTurnRight"){
					  cmdcIndex = MoveBase::CarTurnRight;
				  }else if (Topic == "CarBack"){
					  cmdcIndex = MoveBase::CarBack;
				  }else if (Topic == "CarStop"){
					  cmdcIndex = MoveBase::CarStop;
				  }else if (Topic == "CarUpDataPoint"){
					  cmdcIndex = MoveBase::CarUpDataPoint;
				  }else if (Topic == "CarUpDataLine"){
					  cmdcIndex = MoveBase::CarUpDataLine;
				  }else if (Topic == "CarToStopPoint"){
					  cmdcIndex = MoveBase::CarToStopPoint;
				  }else if (Topic == "CarToTurnAngle"){
					  cmdcIndex = MoveBase::CarToTurnAngle;
				  }else if (Topic == "CarCharge"){
					  cmdcIndex = MoveBase::CarCharge;
				  }else if (Topic == "CarGoHome"){
					  cmdcIndex = MoveBase::CarGoHome;
				  }else if (Topic == "CarTyreDiameter"){
					  cmdcIndex = MoveBase::CarTyreDiameter;
				  }else if (Topic == "CarLampControl"){
					  cmdcIndex = MoveBase::CarLampControl;
				  }else if (Topic == "CarRadarDistance"){
					  cmdcIndex = MoveBase::CarRadarDistance;
				  }else if (Topic == "CarMaxSpeed"){
					  cmdcIndex = MoveBase::CarMaxSpeed;
				  }else if (Topic == "CarCollisionAvoidance"){
					  cmdcIndex = MoveBase::CarCollisionAvoidance;
				  }else if (Topic == "CarClearPassDone"){
					  cmdcIndex = MoveBase::CarClearPassDone;
				  }else if (Topic == "CarSuspendInspect"){
					  cmdcIndex = MoveBase::CarSuspendInspect;
				  }else if (Topic == "CarRecoverInspect"){
					  cmdcIndex = MoveBase::CarRecoverInspect;
				  }else if (Topic == "CarEnablePointAndLine"){
					  cmdcIndex = MoveBase::CarEnablePointAndLine;
				  }else if (Topic == "CarPowerAmount"){
					  cmdcIndex = MoveBase::CarPowerAmount;
				  }
				  //ROS_INFO("cmdcIndex = %s",dic_cmdc.find(Topic)->first.c_str());

				  std_msgs::String cmd_string;
				  cmd_string.data = Topic + ":" + root["Param"].asString();
				  cmd_pub.publish(cmd_string);

				  switch(cmdcIndex){
				  case CarGo:
					  {
						  ROS_INFO("CarGo");
						  std::string Param = root["Param"].asString();
						  if(reader.parse(Param,Paramroot)){
							  double linear = Paramroot["LineSpeed"].asDouble();
							  double angular = Paramroot["AngleSpeed"].asDouble();
							  CarGo_process(linear,angular);
						  }

						  new_item_1["ErrorCode"] = 0;
						  new_item_1["Resultdata"] = "";
						  Json::FastWriter writer;
						  std::string ResultStr = writer.write(new_item_1);
						  new_item["Token"] = Token;
						  new_item["Topic"] = Topic;
						  new_item["SessionId"] = SessionId;
						  time_t t;  //秒时间
						  t = time(NULL); //获取目前秒时间
						  new_item["Time"] = (Json::Int64)t;
						  new_item["Param"] = "";
						  new_item["Result"] = ResultStr;
						  returnStr = writer.write(new_item);
						  //control_style = 1;
					  }
					  break;
				  case CarTurnLeft:
					  {
						  std::string Param = root["Param"].asString();
						  if(reader.parse(Param,Paramroot)){
							  double linear = Paramroot["LineSpeed"].asDouble();
							  double angular = Paramroot["AngleSpeed"].asDouble();
							  CarTurnLeft_process(linear,angular);
						  }
						  new_item_1["ErrorCode"] = 0;
						  new_item_1["Resultdata"] = "";
						  Json::FastWriter writer;
						  std::string ResultStr = writer.write(new_item_1);
						  new_item["Token"] = Token;
						  new_item["Topic"] = Topic;
						  new_item["SessionId"] = SessionId;
						  time_t t;  //秒时间
						  t = time(NULL); //获取目前秒时间
						  new_item["Time"] = (Json::Int64)t;
						  new_item["Param"] = "";
						  new_item["Result"] = ResultStr;
						  returnStr = writer.write(new_item);
						  //control_style = 1;
					  }
					  break;
				  case CarTurnRight:
					  {
						  std::string Param = root["Param"].asString();
						  if(reader.parse(Param,Paramroot)){
							  double linear = Paramroot["LineSpeed"].asDouble();
							  double angular = Paramroot["AngleSpeed"].asDouble();
							  CarTurnRight_process(linear,angular);
						  }
						  new_item_1["ErrorCode"] = 0;
						  new_item_1["Resultdata"] = "";
						  Json::FastWriter writer;
						  std::string ResultStr = writer.write(new_item_1);
						  new_item["Token"] = Token;
						  new_item["Topic"] = Topic;
						  new_item["SessionId"] = SessionId;
						  time_t t;  //秒时间
						  t = time(NULL); //获取目前秒时间
						  new_item["Time"] = (Json::Int64)t;
						  new_item["Param"] = "";
						  new_item["Result"] = ResultStr;
						  returnStr = writer.write(new_item);
						  //control_style = 1;
					  }
					  break;
				  case CarBack:
					  {
						  std::string Param = root["Param"].asString();
						  if(reader.parse(Param,Paramroot)){
							  double linear = Paramroot["LineSpeed"].asDouble();
							  double angular = Paramroot["AngleSpeed"].asDouble();
							  CarBack_process(linear,angular);
						  }
						  new_item_1["ErrorCode"] = 0;
						  new_item_1["Resultdata"] = "";
						  Json::FastWriter writer;
						  std::string ResultStr = writer.write(new_item_1);
						  new_item["Token"] = Token;
						  new_item["Topic"] = Topic;
						  new_item["SessionId"] = SessionId;
						  time_t t;  //秒时间
						  t = time(NULL); //获取目前秒时间
						  new_item["Time"] = (Json::Int64)t;
						  new_item["Param"] = "";
						  new_item["Result"] = ResultStr;
						  returnStr = writer.write(new_item);
						  //control_style = 1;
					  }
					  break;
				  case CarStop:
					  {
						  std::string Param = root["Param"].asString();
						  ros::Duration  rate;
						  rate = ros::Duration(1.0);
						  CarStop_process(0.0,0.0);
//						  rate.sleep();
//						  CarStop_process(0.0,0.0);
//						  if(reader.parse(Param,Paramroot)){
//							  double linear = Paramroot["LineSpeed"].asDouble();
//							  double angular = Paramroot["AngleSpeed"].asDouble();
//
//						  }
						  new_item_1["ErrorCode"] = 0;
						  new_item_1["Resultdata"] = "";
						  Json::FastWriter writer;
						  std::string ResultStr = writer.write(new_item_1);
						  new_item["Token"] = Token;
						  new_item["Topic"] = Topic;
						  new_item["SessionId"] = SessionId;
						  time_t t;  //秒时间
						  t = time(NULL); //获取目前秒时间
						  new_item["Time"] = (Json::Int64)t;
						  new_item["Param"] = "";
						  new_item["Result"] = ResultStr;
						  returnStr = writer.write(new_item);
					  }
					  break;
				  case CarSuspendInspect:
					  {
						  ROS_ERROR("CarSuspendInspect");
						  ros::Duration  rate;
						  rate = ros::Duration(1.0);
						  CarStop_process(0.0,0.0);
//						  rate.sleep();
//						  CarStop_process(0.0,0.0);
//						  if(reader.parse(Param,Paramroot)){
//							  //double linear = Paramroot["LineSpeed"].asDouble();
//							  //double angular = Paramroot["AngleSpeed"].asDouble();
//						  }
						  new_item_1["ErrorCode"] = 0;
						  new_item_1["Resultdata"] = "";
						  Json::FastWriter writer;
						  std::string ResultStr = writer.write(new_item_1);
						  new_item["Token"] = Token;
						  new_item["Topic"] = Topic;
						  new_item["SessionId"] = SessionId;
						  time_t t;  //秒时间
						  t = time(NULL); //获取目前秒时间
						  new_item["Time"] = (Json::Int64)t;
						  new_item["Param"] = "";
						  new_item["Result"] = ResultStr;
						  returnStr = writer.write(new_item);
					  }
					  break;
				  case CarRecoverInspect:
					  {
						  ROS_ERROR("CarRecoverInspect");
//						  if(gohomestate == true && _batteryinfo.Battery_Power < 30){
//							  return;
//						  }
//						  ros::Duration  rate;
//						  rate = ros::Duration(1.0);
//						  CarStop_process(0.0,0.0);
//						  rate.sleep();
//						  rate.sleep();
//						  control_style = "auto";
//						  std_msgs::String style;
//						  style.data = control_style;
//						  control_style_pub.publish(style);
//						  rate.sleep();
//						  rate.sleep();
//						  gohomestate = false;
//						  std::string Param = root["Param"].asString();
//						  if(reader.parse(Param,Paramroot)){
//							  StopPoint_ = Paramroot["StopID"].asString();
//							  //tc_->setStopPoint(StopPoint_);
//						  }
//						  m_msg_info.client_hdl = p_msg_info->client_hdl;
//						  m_msg_info.message = p_msg_info->message;
//						  if( (Point_table.size() == 0) || (Line_table.size() == 0)){
//							  ROS_ERROR("CarToStopPoint (Line_table.size() = %d || Point_table.size() = %d)",Point_table.size(),Line_table.size());
//							  rate.sleep();
//							  return;
//						  }
//
//						  if(Gostoppoint(StopPoint_) ==0){
//							  try{
//								  //boost::shared_lock<boost::mutex> lock(tc_lock_info_);
//								  tc_->setTaskPointID(StopPoint_);
//								  //lock.unlock();
//							  }catch(std::exception& error){
//								  ROS_LOG(ros::console::levels::Level::Error,"CarToStopPoint tc_ setTaskPointID error %s",error.what());
//							  }
//						  }

						  new_item_1["ErrorCode"] = 0;
						  new_item_1["Resultdata"] = StopPoint_;
						  Json::FastWriter writer;
						  std::string ResultStr = writer.write(new_item_1);
						  new_item["Token"] = Token;
						  new_item["Topic"] = Topic;
						  new_item["SessionId"] = SessionId;
						  time_t t;  //秒时间
						  t = time(NULL); //获取目前秒时间
						  new_item["Time"] = (Json::Int64)t;
						  new_item["Param"] = "";
						  new_item["Result"] = ResultStr;
						  returnStr = writer.write(new_item);
					  }
					  break;
				  case CarClearPassDone:
					  {
						  ROS_ERROR("CarClearPassDone");
						  PathDone = "";
					  }
					  break;
				  case CarUpDataPoint:
					  {
						  HasPointList = true;
						  ROS_ERROR("CarUpDataPoint");
						  boost::unique_lock<boost::mutex> lock(point_table_send_mutex_);
						  //ROS_ERROR("CarUpDataPoint");
						  Point_table.clear();
						  Point_table_origin.clear();
						  std::string Param = root["Param"].asString();
						  if(reader.parse(Param,Paramroot)){
							  std::string Pointrow_str = Paramroot["Points"].asString();
							  Json::Value Point_Row;
							  if(reader.parse(Pointrow_str,Point_Row)){
								  int Point_Row_Size = Point_Row.size();
								  std::cout << "Point_Row.size():" << Point_Row.size() << std::endl;

								  for(int i=0 ;i < Point_Row_Size; i++){
									  std::string id = Point_Row[i]["id"].asString();
									  std::cout << "id:" << id << std::endl;
									  std::string fname = Point_Row[i]["fname"].asString();
									  float lasermapX = Point_Row[i]["lasermapX"].asFloat();
									  float lasermapY = Point_Row[i]["lasermapY"].asFloat();;
									  float lasermapTh = Point_Row[i]["lasermapTh"].asFloat();
									  float angle = Point_Row[i]["angle"].asFloat();
									  std::string direction = Point_Row[i]["direction"].asString();
									  int x = Point_Row[i]["x"].asInt();
									  int y = Point_Row[i]["y"].asInt();
									  std::string fun = Point_Row[i]["fun"].asString();
									  bool enable = Point_Row[i]["enable"].asBool();
									  std::string robotid = Point_Row[i]["robotid"].asString();
									  table_point_row row(id,fname,lasermapX,lasermapY,lasermapTh, angle,
											  direction,x, y, fun, enable, robotid);
									  Point_table.insert(point_table_dic::value_type(id,row));
									  Point_table_origin.insert(point_table_dic::value_type(id,row));
								  }
								  point_table_dic::iterator iter = Point_table.begin();
								  for(;iter != Point_table.end();iter++){
									std::cout<<"point_ID:" << iter->second.id << "---lasermapX:" << iter->second.lasermapX << std::endl;
								  }
							  }
						  }
					  }
					  break;
				  case CarUpDataLine:
					  {
						  HasLineList = true;
						  ROS_ERROR("CarUpDataLine");
						  boost::unique_lock<boost::mutex> lock(line_table_send_mutex_);
						  Line_table.clear();
						  Line_table_origin.clear();
						  std::string Param = root["Param"].asString();
						  if(reader.parse(Param,Paramroot)){
							  std::string Linesrow_str = Paramroot["Lines"].asString();
							  Json::Value Lines_Row;
							  if(reader.parse(Linesrow_str,Lines_Row)){
								  int Lines_Row_Size = Lines_Row.size();
								  std::cout << "Lines_Row.size():" << Lines_Row.size() << std::endl;

								  for(int i=0 ;i < Lines_Row_Size; i++){
									  std::string id = Lines_Row[i]["id"].asString();
									  std::cout << "id:" << id << std::endl;
									  std::string point1 = Lines_Row[i]["point1"].asString();
									  std::string point2 = Lines_Row[i]["point2"].asString();
									  float distance = Lines_Row[i]["distance"].asFloat();
									  float turnangle = Lines_Row[i]["turnangle"].asFloat();
									  std::string type = Lines_Row[i]["type"].asString();
									  int order = Lines_Row[i]["order"].asInt();
									  bool enable = Lines_Row[i]["enable"].asBool();;
									  std::string robotid = Lines_Row[i]["robotid"].asString();
									  table_line_row row(id, point1, point2, distance, turnangle, type, enable, robotid, order);
									  Line_table.insert(line_table_dic::value_type(id,row));
									  Line_table_origin.insert(line_table_dic::value_type(id,row));
								  }
							  }

							  carbot_msgs::IDPose pose;
							  carbot_msgs::IDPoseArray task_path_list;
							  line_table_dic::iterator line_it;
							  point_table_dic::iterator point_it;
							  ros::Duration rate;
							  rate = ros::Duration(1.0);
							  ROS_ERROR("Table_list_pub 1");
							  if(Point_table.size() == 0 || Point_table_origin.size() == 0){
								  HasPointList = false;
								  return;
							  }
							  if(Line_table.size() == 0 || Line_table_origin.size() == 0){
								  HasLineList = false;
								  return;
							  }
							  ROS_ERROR("Table_list_pub 2");
							  int cou =0;
							  for(line_it = Line_table_origin.begin();line_it!= Line_table.end(); line_it++){
								  pose.poseid_end =  line_it->second.point2;
								  pose.poseid_start = line_it->second.point1;
								  pose.line_id = line_it->first;
								  pose.curvity = line_it->second.turnangle;

								  pose.pose_corner = Point_table_origin.find(pose.poseid_end)->second.fun;
								  pose.pose.position.x = Point_table_origin.find(pose.poseid_end)->second.lasermapX;
								  pose.pose.position.y = Point_table_origin.find(pose.poseid_end)->second.lasermapY;
								  pose.pose.orientation.w = tf::createQuaternionFromYaw(Point_table_origin.find(pose.poseid_end)->second.lasermapTh).w();
								  pose.pose.orientation.x = tf::createQuaternionFromYaw(Point_table_origin.find(pose.poseid_end)->second.lasermapTh).x();
								  pose.pose.orientation.y = tf::createQuaternionFromYaw(Point_table_origin.find(pose.poseid_end)->second.lasermapTh).y();
								  pose.pose.orientation.z = tf::createQuaternionFromYaw(Point_table_origin.find(pose.poseid_end)->second.lasermapTh).z();

								  pose.pose_start.position.x = Point_table_origin.find(pose.poseid_start)->second.lasermapX;
								  pose.pose_start.position.y = Point_table_origin.find(pose.poseid_start)->second.lasermapY;
								  pose.pose_start.orientation.w = tf::createQuaternionFromYaw(Point_table_origin.find(pose.poseid_start)->second.lasermapTh).w();
								  pose.pose_start.orientation.x = tf::createQuaternionFromYaw(Point_table_origin.find(pose.poseid_start)->second.lasermapTh).x();
								  pose.pose_start.orientation.y = tf::createQuaternionFromYaw(Point_table_origin.find(pose.poseid_start)->second.lasermapTh).y();
								  pose.pose_start.orientation.z = tf::createQuaternionFromYaw(Point_table_origin.find(pose.poseid_start)->second.lasermapTh).z();
								  ROS_ERROR("Table_list_pub enable=%u,line_id=%s,point1=%s,point2=%s",line_it->second.enable,pose.line_id.c_str(),pose.poseid_start.c_str(),pose.poseid_end.c_str());
								  task_path_list.idposearray.push_back(pose);
							  }
							  Table_list_pub.publish(task_path_list);
							  ROS_ERROR("Table_list_pub");
						  }
					  }
					  break;
				  case CarToStopPoint:
					  {
						  ROS_INFO("CarToStopPoint");
						  if(gohomestate == true && _batteryinfo.Battery_Power <= PowerAm){
							  return;
						  }
						  if(BC_recharge == true){
							  return;
						  }
						  ros::Duration  rate;
						  rate = ros::Duration(1.0);
						  CarStop_process(0.0,0.0);
						  rate.sleep();
						  rate.sleep();
						  control_style = "auto";
						  std_msgs::String style;
						  style.data = control_style;
						  control_style_pub.publish(style);
						  rate.sleep();
						  rate.sleep();
						  std::string Param = root["Param"].asString();
						  if(reader.parse(Param,Paramroot)){
							  StopPoint_ = Paramroot["StopID"].asString();
							  //tc_->setStopPoint(StopPoint_);
						  }
						  m_msg_info.client_hdl = p_msg_info->client_hdl;
						  m_msg_info.message = p_msg_info->message;
						  if( (Point_table.size() == 0) || (Line_table.size() == 0)){
							  ROS_ERROR("CarToStopPoint (Line_table.size() = %d || Point_table.size() = %d)",Point_table.size(),Line_table.size());
							  rate.sleep();
							  return;
						  }
						  ROS_ERROR("Bepassinground = %u CarToStopPoint +++++++++++++++++++++", Bepassinground);
						  if(Gostoppoint(StopPoint_,Bepassinground) == 0){
							  try{
								  //boost::shared_lock<boost::mutex> lock(tc_lock_info_);
								  tc_->setStopPointID(StopPoint_);
								  //lock.unlock();
							  }catch(std::exception& error){
								  ROS_LOG(ros::console::levels::Level::Error,"CarToStopPoint tc_ setTaskPointID error %s",error.what());
							  }
							  new_item_1["ErrorCode"] = 0;
							  new_item_1["Resultdata"] = StopPoint_;
							  Json::FastWriter writer;
							  std::string ResultStr = writer.write(new_item_1);
							  new_item["Token"] = Token;
							  new_item["Topic"] = Topic;
							  new_item["SessionId"] = SessionId;
							  time_t t;  //秒时间
							  t = time(NULL); //获取目前秒时间
							  new_item["Time"] = (Json::Int64)t;
							  new_item["Param"] = "";
							  new_item["Result"] = ResultStr;
							  returnStr = writer.write(new_item);
						  }else{
							  new_item_1["ErrorCode"] = 1;
							  new_item_1["Resultdata"] = StopPoint_;
							  Json::FastWriter writer;
							  std::string ResultStr = writer.write(new_item_1);
							  new_item["Token"] = Token;
							  new_item["Topic"] = Topic;
							  new_item["SessionId"] = SessionId;
							  time_t t;  //秒时间
							  t = time(NULL); //获取目前秒时间
							  new_item["Time"] = (Json::Int64)t;
							  new_item["Param"] = "";
							  new_item["Result"] = ResultStr;
							  returnStr = writer.write(new_item);
						  }
					  }
					  break;
				  case CarToTurnAngle:
					  {
						  ROS_INFO("CarToTurnAngle");
						  control_style = "auto";
						  std_msgs::String style;
						  style.data = control_style;
						  control_style_pub.publish(style);
					  }
					  break;
				  case CarCharge:
					  {
						  ROS_INFO("CarCharge");
					  }
					  break;
				  case CarGoHome:
					  {
						  ROS_LOG(ros::console::levels::Level::Info,"CarGoHome","");
						  ROS_WARN("command_parse in CarGoHome");
						  if(gohomestate == true || BC_recharge == true){
							  return;
						  }
						  ros::Duration rate;
						  rate = ros::Duration(1.0);
						  CarStop_process(0.0,0.0);
						  rate.sleep();
						  rate.sleep();
						  control_style = "auto";
						  std_msgs::String style;
						  style.data = control_style;
						  control_style_pub.publish(style);
						  rate.sleep();
						  rate.sleep();
						  gohomestate = true;
						  _InspectInfo.MoveState = "gohome";
						  StopPoint_ = DockPile;
						  m_msg_info.client_hdl = p_msg_info->client_hdl;
						  m_msg_info.message = p_msg_info->message;

						  if( (Point_table.size() == 0) || (Line_table.size() == 0)){
							  ROS_ERROR("CarToStopPoint (Line_table.size() = %d || Point_table.size() = %d)",Point_table.size(),Line_table.size());
							  rate.sleep();
							  gohomestate = false;
							  return;
						  }
						  ROS_ERROR("Bepassinground = %u Gostoppoint Gohome +++++++++++++++++++++", Bepassinground);
						  if(Gostoppoint(StopPoint_,Bepassinground) == 0){
							  try{
								  //boost::shared_lock<boost::mutex> lock(tc_lock_info_);
								  tc_->setStopPointID(StopPoint_);
								  MoveState = "gohome";
								  //lock.unlock();
							  }catch(std::exception& error){
								  ROS_LOG(ros::console::levels::Level::Error,"CarToStopPoint tc_ setTaskPointID error %s",error.what());
							  }
							  new_item_1["ErrorCode"] = 0;
							  new_item_1["Resultdata"] = StopPoint_;
							  Json::FastWriter writer;
							  std::string ResultStr = writer.write(new_item_1);
							  new_item["Token"] = Token;
							  new_item["Topic"] = Topic;
							  new_item["SessionId"] = SessionId;
							  time_t t;  //秒时间
							  t = time(NULL); //获取目前秒时间
							  new_item["Time"] = (Json::Int64)t;
							  new_item["Param"] = "";
							  new_item["Result"] = ResultStr;
							  returnStr = writer.write(new_item);
						  }else{
							  new_item_1["ErrorCode"] = 1;
							  new_item_1["Resultdata"] = StopPoint_;
							  Json::FastWriter writer;
							  std::string ResultStr = writer.write(new_item_1);
							  new_item["Token"] = Token;
							  new_item["Topic"] = Topic;
							  new_item["SessionId"] = SessionId;
							  time_t t;  //秒时间
							  t = time(NULL); //获取目前秒时间
							  new_item["Time"] = (Json::Int64)t;
							  new_item["Param"] = "";
							  new_item["Result"] = ResultStr;
							  returnStr = writer.write(new_item);
						  }
					  }
					  break;
				  case CarTyreDiameter:
					  {
						  ROS_INFO("CarTyreDiameter");
//						  std::string Param = root["Param"].asString();
//						  float max_speed;
//						  if(reader.parse(Param,Paramroot)){
//							  max_speed = Paramroot["MaxSpeed"].asFloat();
//							  ROS_INFO("max_speed = %f",max_speed);
//						  }
//						  new_item_1["ErrorCode"] = 0;
//						  new_item_1["Resultdata"] = "";
//						  Json::FastWriter writer;
//						  std::string ResultStr = writer.write(new_item_1);
//						  new_item["Token"] = Token;
//						  new_item["Topic"] = Topic;
//						  new_item["SessionId"] = SessionId;
//						  time_t t;  //秒时间
//						  t = time(NULL); //获取目前秒时间
//						  new_item["Time"] = (Json::Int64)t;
//						  new_item["Param"] = "";
//						  new_item["Result"] = ResultStr;
//						  returnStr = writer.write(new_item);
//						  private_nh.setParam("/move_base/TrajectoryPlannerROS/max_vel_x",max_speed);
					  }
					  break;
				  case CarPowerAmount:
				  	  {
				  		  ROS_INFO("CarPowerAmount");
						  std::string Param = root["Param"].asString();
						  if(reader.parse(Param,Paramroot)){
							  PowerAm = Paramroot["PowerAm"].asInt();
							  ROS_INFO("PowerAm = %f",PowerAm);
						  }
						  new_item_1["ErrorCode"] = 0;
						  new_item_1["Resultdata"] = "";
						  Json::FastWriter writer;
						  std::string ResultStr = writer.write(new_item_1);
						  new_item["Token"] = Token;
						  new_item["Topic"] = Topic;
						  new_item["SessionId"] = SessionId;
						  time_t t;  //秒时间
						  t = time(NULL); //获取目前秒时间
						  new_item["Time"] = (Json::Int64)t;
						  new_item["Param"] = "";
						  new_item["Result"] = ResultStr;
						  returnStr = writer.write(new_item);
						  private_nh.setParam("/move_base/PowerAm",PowerAm);
				  	  }
				  	  break;
				  case CarLampControl:
					  {
						  ROS_INFO("CarLampControl");
						  try{
							  std::string Param = root["Param"].asString();
							  std::string color;
							  if(reader.parse(Param,Paramroot)){
								   color = Paramroot["LampColor"].asString();
								   ROS_INFO("color = %s",color.c_str());
							  }
							  new_item_1["ErrorCode"] = 0;
							  new_item_1["Resultdata"] = "";
							  Json::FastWriter writer;
							  std::string ResultStr = writer.write(new_item_1);
							  new_item["Token"] = Token;
							  new_item["Topic"] = Topic;
							  new_item["SessionId"] = SessionId;
							  time_t t;  //秒时间
							  t = time(NULL); //获取目前秒时间
							  new_item["Time"] = (Json::Int64)t;
							  new_item["Param"] = "";
							  new_item["Result"] = ResultStr;
							  returnStr = writer.write(new_item);
							  //control_style = 1;
							  carbot_msgs::LedMsg ledcolor;
							  geometry_msgs::Twist target_speed;

							  if(color == "green")
							  {
								  ledcolor.colors = "green";
								  ledcolor.twinkle = false;
							  }
							  if(color == "red")
							  {
								  ledcolor.colors = "red";
								  ledcolor.twinkle = false;
							  }
							  if(color == "yellow")
							  {
								  ledcolor.colors = "yellow";
								  ledcolor.twinkle = false;
							  }

							  led_command_pub.publish(ledcolor);
						  }catch(std::exception& error){
							  ROS_LOG(ros::console::levels::Level::Error,"CarLampControl error %s",error.what());
						  }
					  }
					  break;
				  case CarRadarDistance:
					  {
						  ROS_INFO("CarRadarDistance");

					  }
					  break;
				  case CarMaxSpeed:
					  {
						  ROS_INFO("CarMaxSpeed");
						  std::string Param = root["Param"].asString();
						  float max_speed;
						  if(reader.parse(Param,Paramroot)){
							  max_speed = Paramroot["MaxSpeed"].asFloat();
							  ROS_INFO("max_speed = %f",max_speed);
						  }
						  new_item_1["ErrorCode"] = 0;
						  new_item_1["Resultdata"] = "";
						  Json::FastWriter writer;
						  std::string ResultStr = writer.write(new_item_1);
						  new_item["Token"] = Token;
						  new_item["Topic"] = Topic;
						  new_item["SessionId"] = SessionId;
						  time_t t;  //秒时间
						  t = time(NULL); //获取目前秒时间
						  new_item["Time"] = (Json::Int64)t;
						  new_item["Param"] = "";
						  new_item["Result"] = ResultStr;
						  returnStr = writer.write(new_item);
						  private_nh.setParam("/move_base/TrajectoryPlannerROS/max_vel_x",max_speed);
					  }
					  break;
				  case CarCollisionAvoidance:
					  {
						  ROS_INFO("CarCollisionAvoidance");
					  }
					  break;
				  case CarEnablePointAndLine:
				  	  {
				  		  ROS_ERROR("CarEnablePointAndLine");
				  		  ros::Duration rate;
						  rate = ros::Duration(1.0);
						  std::string Param = root["Param"].asString();
						  ROS_ERROR("CarEnablePointAndLine 11");
						  if(reader.parse(Param,Paramroot))
						  {
							  std::string RobotID = Paramroot["RobotID"].asString();
							  ROS_ERROR("Paramroot[RobotID]=%s",RobotID.c_str());
							  Json::Value PointList_Value = Paramroot["EnableCarPointList"];
							  Json::Value LineList_Value = Paramroot["EnableCarLineList"];
							  if( (Point_table.size() == 0) || (Line_table.size() == 0)){
								  ROS_ERROR("CarEnablePointAndLine (Line_table.size() = %d || Point_table.size() = %d)",Point_table.size(),Line_table.size());
								  rate.sleep();
								  return;
							  }
							  for(int i=0 ; i < PointList_Value.size(); i++)
							  {
								  std::string pointlist_id = PointList_Value[i]["ID"].asString();
								  bool point_enable = PointList_Value[i]["Enable"].asBool();
								  Point_table.find(pointlist_id)->second.enable = point_enable;
								  Point_table_origin.find(pointlist_id)->second.enable = point_enable;
								  ROS_WARN("pointlist_id:%s,enable:%u",pointlist_id.c_str(),point_enable);
							  }

							  for(int i=0 ; i < LineList_Value.size(); i++)
							  {
								  std::string linelist_id = LineList_Value[i]["ID"].asString();
								  bool line_enable = LineList_Value[i]["Enable"].asBool();
								  Line_table.find(linelist_id)->second.enable = line_enable;
								  Line_table_origin.find(linelist_id)->second.enable = line_enable;
								  ROS_WARN("linelist_id:%s,enable:%u",linelist_id.c_str(),line_enable);
							  }

							  //ROS_ERROR("Table_list_pub");
							  ROS_ERROR("CarEnablePointAndLine 22");
							  point_table_dic::iterator iter = Point_table.begin();
							  for(;iter != Point_table.end();iter++){
								//std::cout<< "Point_ID:"<< iter->second.id << ";Enable:"<< iter->second.enable << std::endl;
								ROS_ERROR("Point_ID=%s;Enable=%u",(iter->second.id).c_str(),iter->second.enable);
							  }
							  line_table_dic::iterator iterl = Line_table.begin();
							  for(;iterl != Line_table.end();iterl++){
								//std::cout<< "Line_ID:"<< iterl->second.id << ";Enable:"<< iterl->second.enable << std::endl;
								ROS_ERROR("Line_ID=%s;Enable=%u",(iterl->second.id).c_str(),iterl->second.enable);
							  }

							  ros::Duration  rate;
							  rate = ros::Duration(1.0);
							  CarStop_process(0.0,0.0);
							  rate.sleep();
							  rate.sleep();
							  control_style = "auto";
							  std_msgs::String style;
							  style.data = control_style;
							  control_style_pub.publish(style);

							  new_item_1["ErrorCode"] = 0;
							  if(Point_table.find(StopPoint_) !=  Point_table.end()){
								  new_item_1["Resultdata"] = StopPoint_;
							  }else{
								  new_item_1["Resultdata"] = "Notask";
							  }
							  Json::FastWriter writer;
							  std::string ResultStr = writer.write(new_item_1);
							  new_item["Token"] = Token;
							  new_item["Topic"] = Topic;
							  new_item["SessionId"] = SessionId;
							  time_t t;  //秒时间
							  t = time(NULL); //获取目前秒时间
							  new_item["Time"] = (Json::Int64)t;
							  new_item["Param"] = "delete";
							  new_item["Result"] = ResultStr;
							  returnStr = writer.write(new_item);
						  }
					  }
					  break;
				  default:
					  control_style = "auto";
					  std_msgs::String style;
					  style.data = control_style;
					  control_style_pub.publish(style);
					  break;
				  }

			  try{
				  if(_hdl.lock().get() > 0)
				  {
					  std::cout << _hdl.lock().get() << std::endl;
					  echo_server.send(_hdl, returnStr,websocketpp::frame::opcode::value::text);
					  std::cout<< "cmd_feedback_send------" << "on_message called with hdl:" << _hdl.lock().get()
								  << " and message: " << returnStr << std::endl;
				  }
				  }catch(std::exception& error){
					  ROS_LOG(ros::console::levels::Level::Error,"InspectInfo_rec error %s",error.what());
				  }
			  }
    	  }
    	  catch (...) {
    		  std::cout<< "command_exception------" << "on_message called with hdl: " << (p_msg_info->client_hdl).lock().get()
					<< " and message: " << (p_msg_info->message)->get_payload()
					<< std::endl;
		  }
    	  //handle.detach();
    	  return;
      }

      boost::mutex tc_lock_info_;

      void process_state_rec(const std_msgs::Int32 & process_state){
    	  task_process_state_ = process_state.data;
      }

      bool delaymotionService(carbot_msgs::DelayactionRequest& req, carbot_msgs::DelayactionResponse& resp)
      {
    	  ros::Duration rate;
    	  rate = ros::Duration(1.0);
    	  if(req.Overtime){
    		  //标记计算下一组路径为绕行路径
    		  Bepassinground = true;
    		  tc_->setBepassinground(Bepassinground);
    		  rate.sleep();
    		  rate.sleep();
    		  Json::Value new_item,new_item_1,new_item_2;
    		  std::string returnStr;
    		  new_item_1["ErrorCode"] = 0;
    		  new_item_1["Resultdata"] = req.Task_goal;
    		  Json::FastWriter writer;
    		  std::string ResultStr = writer.write(new_item_1);
    		  new_item_2["OverTime"] = req.Limit_time;
    		  std::string ParamStr = writer.write(new_item_2);
    		  new_item["Token"] = "";
    		  new_item["Topic"] = "ObstacleOverTime";
    		  boost::uuids::uuid uuid = boost::uuids::random_generator()();
    		  std::string struuid = boost::lexical_cast<std::string>(uuid);
    		  new_item["SessionId"] = struuid;
    		  time_t t;  //秒时间
    		  t = time(NULL); //获取目前秒时间
    		  new_item["Time"] = (Json::Int64)t;
    		  new_item["Param"] = ParamStr;
    		  new_item["Result"] = ResultStr;
    		  returnStr = writer.write(new_item);

    		  try{
    			  if(_hdl.lock().get() > 0){
    				  echo_server.send(_hdl, returnStr,websocketpp::frame::opcode::value::text);
    				  std::cout<< "delaymotionService---------------" << "on_message called with hdl:" << _hdl.lock().get()
    						  << " and message: " << returnStr << std::endl;
    			  }
    		  }catch(std::exception& error){
    			  ROS_LOG(ros::console::levels::Level::Error,"delaymotion error %s",error.what());
    		  }

    		  resp.Delay_motion_state = true;
    		  //_InspectInfo.Bepassinground = true;
    		  ROS_ERROR("delay_motion success +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    	  }else{
    		  resp.Delay_motion_state = false;
    		  ROS_ERROR("delay_motion false -----------------------------------------------------------------------------------------------------------");
    	  }
    	  return true;
      }

	  /*******************************************************
	   *int32 RuntimeLength #运行时长 单位:分钟
	   *int32 OperationMileage #运行里程 单位：米
	   *bool HasPointList #点表数据
	   *bool HasLineList #线表数据
	   *string PathDone #已走过停车点位集合
	   *string StartPoint #当前任务起始点
	   *string EndPoint #当前任务终止点
	   *float32 PosX #当前定位坐标X轴
	   *float32 PosY #当前定位坐标Y轴
	   *float64 Angle #当前定位坐标方位角
	   *string CurrentPoint #当前任务点
	   *string LineID #当前所在拓扑线段
	   *float32 LinePercent #当前拓扑线段比例 单位:百分比
	   *string MoveState #运行状态
	   *int32 InCtrlArea #正常运行区域内 0-在区域内 1-在区域外 2-无法定位
	   *************************************************************/
      void InspectInfo_rec(const carbot_msgs::InspectInfo& inspectInfo){
    	  //ROS_INFO("void InspectInfo_rec(const carbot_msgs::inspectInfo& inspectInfo)");

    	  _InspectInfo = inspectInfo;
		  nextpoint_ = _InspectInfo.EndPoint;
		  currentpoint_ = _InspectInfo.CurrentPoint;
		  if(curgoalpoint_ == "nopoint" || curgoalpoint_ == ""){
			  curgoalpoint_ = currentpoint_;
		  }

		  ROS_ERROR("curgoalpoint_= %s",curgoalpoint_.c_str());
		  ROS_ERROR("currentpoint_= %s",currentpoint_.c_str());
		  ROS_ERROR("nextpoint_= %s",nextpoint_.c_str());

		  Json::Value new_item,new_item_1,new_item_2;
		  std::string returnStr;
		  new_item_1["ErrorCode"] = 0;
		  new_item_1["Resultdata"] = "";
		  Json::FastWriter writer;
		  std::string ResultStr = writer.write(new_item_1);
		  new_item_2["RuntimeLength"] = _InspectInfo.RuntimeLength;
		  new_item_2["OperationMileage"] = _InspectInfo.OperationMileage;
		  new_item_2["HasPointList"] = HasPointList;
		  new_item_2["HasLineList"] = HasLineList;
		  new_item_2["StartPoint"] = _InspectInfo.StartPoint;
		  new_item_2["EndPoint"] = _InspectInfo.EndPoint;

		  if(PathDone == ""){
			  PathDone = _InspectInfo.StartPoint;
		  }

		  new_item_2["PathDone"] = PathDone;
		  new_item_2["PosX"] = _InspectInfo.PosX;
		  new_item_2["PosY"] = _InspectInfo.PosY;
		  new_item_2["Angle"] = _InspectInfo.Angle;
		  new_item_2["CurrentPoint"] = curgoalpoint_;

		  if (_InspectInfo.LineID != ""){
			  new_item_2["LineID"] = _InspectInfo.LineID;
			  new_item_2["LinePercent"] = _InspectInfo.LinePercent;
		  }else{
			  new_item_2["LineID"] = std::string(DockPile + "-"+ DockPoint);
			  _InspectInfo.LineID = std::string(DockPile + "-" + DockPoint);
			  new_item_2["LinePercent"] = 0;;
		  }
//		  if(_InspectInfo.MoveState == "gohome" && curgoalpoint_ == DockPile){
//			  new_item_2["MoveState"] = std::string("stop at " + DockPile);
//			  _InspectInfo.MoveState = std::string("stop at " + DockPile);
//		  }else{
//
//		  }
		  new_item_2["MoveState"] = _InspectInfo.MoveState;
		  new_item_2["InCtrlArea"] = _InspectInfo.InCtrlArea;
		  std::string ParamStr = writer.write(new_item_2);
		  //
		  new_item["Token"] = "";
		  new_item["Topic"] = "InspectInfo";
		  boost::uuids::uuid uuid = boost::uuids::random_generator()();
		  std::string struuid = boost::lexical_cast<std::string>(uuid);
		  new_item["SessionId"] = struuid;
		  time_t t;  //秒时间
		  t = time(NULL); //获取目前秒时间
		  new_item["Time"] = (Json::Int64)t;
		  new_item["Param"] = ParamStr;
		  new_item["Result"] = ResultStr;
		  returnStr = writer.write(new_item);

		  static ros::Time start_time = ros::Time::now();
		  static ros::Time end_time = ros::Time::now();
		  double delay_time = end_time.toSec() - start_time.toSec();

		  if(delay_time > 2){
			  start_time = ros::Time::now();
			  end_time = ros::Time::now();
			  try{
				  if(_hdl.lock().get() > 0)
				  {
					  std::cout << _hdl.lock().get() << std::endl;
					  echo_server.send(_hdl, returnStr,websocketpp::frame::opcode::value::text);
					  std::cout<< "InspectInfo_send------" << "on_message called with hdl:" << _hdl.lock().get()
							  << " and message: " << returnStr << std::endl;
				  }
			  }catch(std::exception& error){
				  ROS_LOG(ros::console::levels::Level::Error,"InspectInfo_rec error %s",error.what());
			  }
		  }else{
			  end_time = ros::Time::now();

		  }
      }

      /*
	    int8 Obt1_Dis #1号超声波距离 单位:cm
		int8 Obt2_Dis #2号超声波距离 单位:cm
		int8 Obt3_Dis #3号超声波距离 单位:cm
		int8 FP1_Status #1号防跌落通信状态 0-在线 1-下线
		int8 FP2_Status #2号防跌落通信状态 0-在线 1-下线
		int16 FP1_Dis #1号防跌落距离 单位:cm
		int16 FP2_Dis #2号防跌落距离 单位:cm
		int8 FC_Alarm #前碰撞触发报警 0-正常 1-报警
		int8 OA_Alarm #前避障触发报警 0-正常 1-报警
		int8 ES_Alarm #急停触发报警 0-正常 1-报警
		int8 FP_Alarm #防跌落触发报警 0-正常 1-报警
		int8 OBT_Status #超声波通信状态 0-在线 1-下线
		int8 LFW_Status #左前电机通信状态 0-在线 1-下线
		int8 RFW_Status #右前电机通信状态 0-在线 1-下线
		int8 LBW_Status #左后电机通信状态 0-在线 1-下线
		int8 RBW_Status #右后电机通信状态 0-在线 1-下线
		int8 Laser_Status #雷达通信状态 0-在线 1-下线
		int8 TEMP_Chassis #车体温度 单位 摄氏度
		int8 LampColor #三色灯颜色 0-关闭 1-红 2-绿 3-黄 4-闪红 5-闪绿 6-闪黄
		int8 ObstacleWarning #障碍物报警 0-无障碍 1-水平激光雷达报警 2- -15度激光雷达报警 3-前超声波避障报警
		float32 RunningSpeed #运动速度 m/s
       */
      void EquipmentInfo_rec(const carbot_msgs::EquipmentInfo& equipmentInfo){
    	  //ROS_INFO("void EquipmentInfo_rec(const carbot_msgs::equipmentInfo& equipmentInfo)");
    	  _equipmentInfo = equipmentInfo;
    	  Json::Value new_item,new_item_1,new_item_2;
    	  std::string returnStr;
		  new_item_1["ErrorCode"] = 0;
		  new_item_1["Resultdata"] = "";
		  Json::FastWriter writer;
		  std::string ResultStr = writer.write(new_item_1);
		  //
		  new_item_2["Obt1_Dis"] = _equipmentInfo.Obt1_Dis;
		  new_item_2["Obt2_Dis"] = _equipmentInfo.Obt2_Dis; //cmd_vel_cur.linear.x;
		  new_item_2["Obt3_Dis"] = _equipmentInfo.Obt3_Dis;

		  new_item_2["FP1_Status"] = _equipmentInfo.FP1_Status;
		  new_item_2["FP2_Status"] = _equipmentInfo.FP2_Status;

		  new_item_2["FP1_Dis"] = _equipmentInfo.FP1_Dis;
		  new_item_2["FP2_Dis"] = _equipmentInfo.FP2_Dis;

		  new_item_2["FC_Alarm"] = _equipmentInfo.FC_Alarm;
		  new_item_2["OA_Alarm"] = _equipmentInfo.OA_Alarm;
		  new_item_2["ES_Alarm"] = _equipmentInfo.ES_Alarm;
		  new_item_2["FP_Alarm"] = _equipmentInfo.FP_Alarm;

		  new_item_2["OBT_Status"] = _equipmentInfo.OBT_Status;
		  new_item_2["LFW_Status"] = _equipmentInfo.LFW_Status;
		  new_item_2["RFW_Status"] = _equipmentInfo.RFW_Status;
		  new_item_2["LBW_Status"] = _equipmentInfo.LBW_Status;
		  new_item_2["RBW_Status"] = _equipmentInfo.RBW_Status;

		  new_item_2["Laser_Status"] = _equipmentInfo.Laser_Status;
		  new_item_2["TEMP_Chassis"] = _equipmentInfo.TEMP_Chassis;
		  new_item_2["LampColor"] = _equipmentInfo.LampColor;
		  new_item_2["ObstacleWarning"] =_equipmentInfo.ObstacleWarning;
		  new_item_2["RunningSpeed"] = _equipmentInfo.RunningSpeed;
		  std::string ParamStr = writer.write(new_item_2);
		  //
		  new_item["Token"] = "";
		  new_item["Topic"] = "EquipmentInfo";
		  boost::uuids::uuid uuid = boost::uuids::random_generator()();
		  std::string struuid = boost::lexical_cast<std::string>(uuid);
		  new_item["SessionId"] = struuid;
		  time_t t;  //秒时间
		  t = time(NULL); //获取目前秒时间
		  new_item["Time"] = (Json::Int64)t;
		  new_item["Param"] = ParamStr;
		  new_item["Result"] = ResultStr;
		  returnStr = writer.write(new_item);

		  static ros::Time start_time = ros::Time::now();
		  static ros::Time end_time = ros::Time::now();
		  double delay_time = end_time.toSec() - start_time.toSec();

		  if(delay_time > 2){
				  start_time = ros::Time::now();
				  end_time = ros::Time::now();
				  try{
					  if(_hdl.lock().get() > 0){
						  std::cout << "_hdl.lock().get() EquipmentInfo_rec " << _hdl.lock().get() << std::endl;
						  echo_server.send(_hdl, returnStr,websocketpp::frame::opcode::value::text);
						  std::cout<< "EquipmentInfo_rec_send------" << "on_message called with hdl:" << _hdl.lock().get()
								  << " and message: " << returnStr << std::endl;
					  }
				  }catch(std::exception& error){
					  ROS_LOG(ros::console::levels::Level::Error, "EquipmentInfo_rec error %s", error.what());
				  }
		  }else{
			  end_time = ros::Time::now();
		  }

		  _carinfo.ChargeState = _batteryinfo.BC_Status;
		  _carinfo.FuselageTemperature = _equipmentInfo.TEMP_Chassis;
		  _carinfo.ChargeVoltage = _batteryinfo.Battery_Volt;
		  _carinfo.Electricity = _batteryinfo.Battery_Power;
		  _carinfo.ChargeElectricCurrent = _batteryinfo.Battery_Current;
		  _carinfo.EmergencyStop = _equipmentInfo.ES_Alarm;
		  _carinfo.LampColor = _equipmentInfo.LampColor;
		  _carinfo.ObstacleWarning = _equipmentInfo.ObstacleWarning;
		  _carinfo.CollisionAvoidance = _equipmentInfo.FC_Alarm;
		  _carinfo.LeftUpWheel = _equipmentInfo.LFW_Status;
		  _carinfo.RightUpWheel = _equipmentInfo.RFW_Status;
		  _carinfo.LeftDownWheel = _equipmentInfo.LBW_Status;
		  _carinfo.RightDownWheel = _equipmentInfo.RBW_Status;
		  _carinfo.CurrentPoint = curgoalpoint_;
		  _carinfo.RunningSpeed = _equipmentInfo.RunningSpeed;
		  _carinfo.RuntimeLength = _InspectInfo.RuntimeLength;
		  _carinfo.OperationMileage = _InspectInfo.OperationMileage;
		  _carinfo.HasPointList = HasPointList;
		  _carinfo.HasLineList = HasLineList;
		  _carinfo.Bepassinground = Bepassinground;
		  All_info_pub.publish(_carinfo);
      }
      /*
	    int8 LFMotor_Status #左前电机状态 #(39 正常)
		int8 RFMotor_Status #右前电机状态
		int8 LBMotor_Status #左后电机状态
		int8 RBMotor_Status #右后电机状态
		int32 LFMotor_Error #左前电机错误 #(6100 车轮过载)(C150 私服驱动器CN2脱落)
		int32 RFMotor_Error #右前电机错误 #(6100 车轮过载)(C150 私服驱动器CN2脱落)
		int32 LBMotor_Error #左后电机错误 #(6100 车轮过载)(C150 私服驱动器CN2脱落)
		int32 RBMotor_Error #右后电机错误 #(6100 车轮过载)(C150 私服驱动器CN2脱落)
       */
      void DriverInfo_rec(const carbot_msgs::DriverInfo& driverInfo){
          _driverInfo = driverInfo;
    	  Json::Value new_item,new_item_1,new_item_2;
    	  std::string returnStr;
		  new_item_1["ErrorCode"] = 0;
		  new_item_1["Resultdata"] = "";
		  Json::FastWriter writer;
		  std::string ResultStr = writer.write(new_item_1);
		  //
		  new_item_2["LFMotor_Status"] = _driverInfo.LFMotor_Status;
		  new_item_2["RFMotor_Status"] = _driverInfo.RFMotor_Status;//cmd_vel_cur.linear.x;
		  new_item_2["LBMotor_Status"] = _driverInfo.LBMotor_Status;
		  new_item_2["RBMotor_Status"] = _driverInfo.RBMotor_Status;

		  new_item_2["LFMotor_Error"] = _driverInfo.LFMotor_Error;
		  new_item_2["RFMotor_Error"] = _driverInfo.RFMotor_Error;
		  new_item_2["LBMotor_Error"] = _driverInfo.LBMotor_Error;
		  new_item_2["RBMotor_Error"] = _driverInfo.RBMotor_Error;
		  std::string ParamStr = writer.write(new_item_2);
		  //
		  new_item["Token"] = "";
		  new_item["Topic"] = "DriverInfo";
		  boost::uuids::uuid uuid = boost::uuids::random_generator()();
		  std::string struuid = boost::lexical_cast<std::string>(uuid);
		  new_item["SessionId"] = struuid;
		  time_t t;  //秒时间
		  t = time(NULL); //获取目前秒时间
		  new_item["Time"] = (Json::Int64)t;
		  new_item["Param"] = ParamStr;
		  new_item["Result"] = ResultStr;
		  returnStr = writer.write(new_item);

		  static ros::Time start_time = ros::Time::now();
		  static ros::Time end_time = ros::Time::now();
		  double delay_time = end_time.toSec() - start_time.toSec();

		  if(delay_time > 2){
				  start_time = ros::Time::now();
				  end_time = ros::Time::now();
				  try{
					  if(_hdl.lock().get() > 0){
						  std::cout << "_hdl.lock().get() DriverInfo_rec " << _hdl.lock().get() << std::endl;
						  echo_server.send(_hdl, returnStr,websocketpp::frame::opcode::value::text);
						  std::cout<< "DriverInfo_rec_send------" << "on_message called with hdl:" << _hdl.lock().get()
								  << " and message: " << returnStr << std::endl;
					  }
				  }catch(std::exception& error){
					  ROS_LOG(ros::console::levels::Level::Error,"DriverInfo_rec error %s",error.what());
				  }
		  }else{
			  end_time = ros::Time::now();
			  delay_time = end_time.toSec() - start_time.toSec();
		  }

      }
      /*
	    int8 Battery_Power #电池电量 单位:百分比
		int16 Battery_Volt #电池电压 单位:伏特
		int16 Battery_Current #放电电流 单位:安培
		int8 Battery_Temp #电池温度 单位:摄氏度
		int8 BC_Status #充电状态 0-放电 1-充电
		int8 BMS_Status #电池在线 0-在线 1-下线
		bool ExternalPowerSupply #是否备用电源
       */
      void BatteryInfo_rec(const carbot_msgs::BatteryInfo& batteryInfo){
          _batteryinfo = batteryInfo;
    	  Json::Value new_item,new_item_1,new_item_2;
    	  std::string returnStr;
		  new_item_1["ErrorCode"] = 0;
		  new_item_1["Resultdata"] = "";
		  Json::FastWriter writer;
		  std::string ResultStr = writer.write(new_item_1);
		  //
		  new_item_2["Battery_Power"] = _batteryinfo.Battery_Power;
		  new_item_2["Battery_Volt"] = _batteryinfo.Battery_Volt;//cmd_vel_cur.linear.x;
		  new_item_2["Battery_Current"] = _batteryinfo.Battery_Current;
		  new_item_2["Battery_Temp"] = _batteryinfo.Battery_Temp;
		  new_item_2["BC_Status"] = _batteryinfo.BC_Status;
		  new_item_2["BMS_Status"] = _batteryinfo.BMS_Status;
		  new_item_2["ExternalPowerSupply"] = _batteryinfo.ExternalPowerSupply;
		  std::string ParamStr = writer.write(new_item_2);
		  //
		  new_item["Token"] = "";
		  new_item["Topic"] = "BatteryInfo";
		  boost::uuids::uuid uuid = boost::uuids::random_generator()();
		  std::string struuid = boost::lexical_cast<std::string>(uuid);
		  new_item["SessionId"] = struuid;
		  time_t t;  //秒时间
		  t = time(NULL); //获取目前秒时间
		  new_item["Time"] = (Json::Int64)t;
		  new_item["Param"] = ParamStr;
		  new_item["Result"] = ResultStr;
		  returnStr = writer.write(new_item);

		  static ros::Time start_time = ros::Time::now();
		  static ros::Time end_time = ros::Time::now();
		  double delay_time = end_time.toSec() - start_time.toSec();

		  if(delay_time > 2){
				  start_time = ros::Time::now();
				  end_time = ros::Time::now();
				  try{
					  if(_hdl.lock().get() > 0){
						  std::cout << "_hdl.lock().get() BatteryInfo_rec " << _hdl.lock().get() << std::endl;
						  echo_server.send(_hdl, returnStr,websocketpp::frame::opcode::value::text);
						  std::cout<< "BatteryInfo_rec_send------" << "on_message called with hdl:" << _hdl.lock().get()
								  << " and message: " << returnStr << std::endl;
					  }
				  }catch(std::exception& error){
					  ROS_LOG(ros::console::levels::Level::Error,"BatteryInfo_rec error %s",error.what());
				  }
		  }else{
			  end_time = ros::Time::now();
			  delay_time = end_time.toSec() - start_time.toSec();
		  }
      }

      void currenttask_rec(const carbot_msgs::IDPose& cur_task)
      {
    	  ROS_INFO("currenttask_rec(const carbot_msgs::IDPose& cur_task)");
    	  try{
    		  tc_->setStartPointID(cur_task.poseid_start);
			  tc_->setEndPointID(cur_task.poseid_end);
			  tc_->setTaskLineID(cur_task.line_id);
			  curtask_goal_id = cur_task.poseid_end;
			  tc_->setTaskPointID(cur_task.poseid_end);
		  }catch(std::exception& error){
			  ROS_LOG(ros::console::levels::Level::Error,"currenttask_rec tc_->setTaskPointID error %s",error.what());
		  }
      }

      void task_arrived(const carbot_msgs::Taskarrived& cur_goal_id){

    	  ROS_ERROR("void task_arrived(const std_msgs::String& stoppoint){");
    	  try{
			  //pose_init_ = cur_goal_id.Request;
			  //tc_->setTaskArrivedID(curgoalpoint_);
			  Bepassinground = cur_goal_id.Bepassinground;
			  BC_Statu = (bool)cur_goal_id.BC_Status;
			  tc_->setBepassinground(Bepassinground);

		      ros::Rate r(0.5);
		      r.sleep();
		      geometry_msgs::PoseWithCovarianceStamped initial_pos;
		      if(cur_goal_id.Taskarrived == DockPile){
		    	  //filling header with relevant information
		    	  initial_pos.header.frame_id = "map";
		    	  initial_pos.header.stamp = ros::Time::now();
		    	  //filling payload with relevant information gathered from subscribing
		    	  // to initialpose topic published by RVIZ via rostopic echo initialpose
		    	  initial_pos.pose.pose.position.x =  _InspectInfo.PosX;//Point_table.find(curgoalpoint_)->second.lasermapX;
		    	  initial_pos.pose.pose.position.y = _InspectInfo.PosY;//Point_table.find(curgoalpoint_)->second.lasermapY;
		    	  initial_pos.pose.pose.position.z = 0.0;

		    	  initial_pos.pose.pose.orientation.x =tf::createQuaternionFromYaw(_InspectInfo.Angle).x();//Quaternion(*quaternion_from_euler(0,0,th,axes='sxyz'))
		    	  initial_pos.pose.pose.orientation.y =tf::createQuaternionFromYaw(_InspectInfo.Angle).y();
		    	  initial_pos.pose.pose.orientation.z =tf::createQuaternionFromYaw(_InspectInfo.Angle).z();
		    	  initial_pos.pose.pose.orientation.w =tf::createQuaternionFromYaw(_InspectInfo.Angle).w();

		    	  for(int i=0;i < 36;i++){
		    		  initial_pos.pose.covariance[i]= 0.0;
		    	  }

		    	  initial_pos.pose.covariance[0] = 0.25;
		    	  initial_pos.pose.covariance[7] = 0.25;
		    	  initial_pos.pose.covariance[35] = 0.06853891945200942;

		    	  if(/*pose_init_ &&*/ cur_goal_id.Request){
		    		  if(_InspectInfo.header.seq != InspectInfo_seq_last){
						  r.sleep();
						  intial_pose_pub.publish(initial_pos);
						  InspectInfo_seq_last = _InspectInfo.header.seq;
  //					  r.sleep();
  //					  intial_pose_pub.publish(initial_pos);
		    		  }else{
		    			  ROS_ERROR("task_arrived initial_pos: _InspectInfo msg connect error:seq %d",_InspectInfo.header.seq);
		    		  }
				  }

		    	  r.sleep();
		    	  carbot_msgs::IDPose pose;
				  carbot_msgs::IDPoseArray task_path;
				  point_table_dic::iterator point_iter;
				  line_table_dic::iterator line_iter;

		    	  if(_batteryinfo.BC_Status == 0 && Charge_count < 4){
		    		  //cz-cd
		    		  pose.line_id = DockPile + "-" + DockPoint;
					  pose.poseid_start = DockPile;
					  pose.poseid_end = DockPoint;
					  point_iter = Point_table.find(DockPoint);
					  line_iter = Line_table.find(pose.line_id);
					  pose.passinground_task = Bepassinground;//任务属性标记绕行性质
					  pose.pose_corner = point_iter->second.fun;
					  pose.curvity = line_iter->second.turnangle;
					  pose.pose.position.x = point_iter->second.lasermapX;
					  pose.pose.position.y = point_iter->second.lasermapY;
					  pose.pose.orientation.w = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).w();
					  pose.pose.orientation.x = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).x();
					  pose.pose.orientation.y = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).y();
					  pose.pose.orientation.z = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).z();//new tf::Quaternion(0,0,Point_table.find(curpoint)->second.lasermapTh);
					  point_iter = Point_table.find(DockPile);
					  pose.pose_start.position.x = point_iter->second.lasermapX;
					  pose.pose_start.position.y = point_iter->second.lasermapY;
					  pose.pose_start.orientation.w = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).w();
					  pose.pose_start.orientation.x = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).x();
					  pose.pose_start.orientation.y = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).y();
					  pose.pose_start.orientation.z = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).z();
					  task_path.idposearray.push_back(pose);
					  //cd-cz
		    		          pose.line_id = DockPoint + "-" + DockPile;
		    		          pose.poseid_start = DockPoint;
		    		          pose.poseid_end = DockPile;
					  line_iter = Line_table.find(pose.line_id);
					  point_iter = Point_table.find(DockPile);
					  pose.passinground_task = Bepassinground;//任务属性标记绕行性质
					  pose.pose_corner = point_iter->second.fun;
					  pose.curvity = line_iter->second.turnangle;
					  pose.pose.position.x = point_iter->second.lasermapX;
					  pose.pose.position.y = point_iter->second.lasermapY;
					  pose.pose.orientation.w = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).w();
					  pose.pose.orientation.x = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).x();
					  pose.pose.orientation.y = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).y();
					  pose.pose.orientation.z = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).z();//new tf::Quaternion(0,0,Point_table.find(curpoint)->second.lasermapTh);
					  point_iter = Point_table.find(DockPoint);
					  pose.pose_start.position.x = point_iter->second.lasermapX;
					  pose.pose_start.position.y = point_iter->second.lasermapY;
					  pose.pose_start.orientation.w = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).w();
					  pose.pose_start.orientation.x = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).x();
					  pose.pose_start.orientation.y = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).y();
					  pose.pose_start.orientation.z = tf::createQuaternionFromYaw(point_iter->second.lasermapTh).z();
					  task_path.idposearray.push_back(pose);
					  task_posearray_pub.publish(task_path);
					  BC_recharge = true;
					  Charge_count++;
					  return;
		    	  }else{
			    	  gohomestate = false;
			          _InspectInfo.MoveState = std::string("stop at " + DockPile);
			    	  PathDone = DockPile;
			    	  BC_recharge = false;
			    	  Charge_count = 0;
		    	  }
		      }
		      else{
				  //filling header with relevant information
				  initial_pos.header.frame_id = "map";
				  initial_pos.header.stamp = ros::Time::now();
				  //filling payload with relevant information gathered from subscribing
				  //to initialpose topic published by RVIZ via rostopic echo initialpose

				  initial_pos.pose.pose.position.x = _InspectInfo.PosX;//Point_table.find(curgoalpoint_)->second.lasermapX;
				  initial_pos.pose.pose.position.y = _InspectInfo.PosY;//Point_table.find(curgoalpoint_)->second.lasermapY;
				  initial_pos.pose.pose.position.z = 0.0;
				  ////tf::createQuaternionFromYaw(Point_table.find(curgoalpoint_)->second.lasermapTh).x();//Quaternion(*quaternion_from_euler(0,0,th,axes='sxyz'))
				  initial_pos.pose.pose.orientation.x =tf::createQuaternionFromYaw(_InspectInfo.Angle).x();
				  initial_pos.pose.pose.orientation.y =tf::createQuaternionFromYaw(_InspectInfo.Angle).y();
				  initial_pos.pose.pose.orientation.z =tf::createQuaternionFromYaw(_InspectInfo.Angle).z();
				  initial_pos.pose.pose.orientation.w =tf::createQuaternionFromYaw(_InspectInfo.Angle).w();

				  for(int i=0;i<36;i++){
					  initial_pos.pose.covariance[i]= 0.0;
				  }

				  initial_pos.pose.covariance[0] = 0.25;
				  initial_pos.pose.covariance[7] = 0.25;
				  initial_pos.pose.covariance[35] = 0.06853891945200942;

				  if(pose_init_ && cur_goal_id.Request){
		    		  if(_InspectInfo.header.seq != InspectInfo_seq_last){
						  r.sleep();
						  intial_pose_pub.publish(initial_pos);
						  InspectInfo_seq_last = _InspectInfo.header.seq;
  //					  r.sleep();
  //					  intial_pose_pub.publish(initial_pos);
		    		  }else{
		    			  ROS_ERROR("task_arrived initial_pos: _InspectInfo msg connect error:seq %d",_InspectInfo.header.seq);
		    		  }
				  }
		      }
		      ROS_ERROR("task_arrived Charge_count: %d", Charge_count);
		      if(BC_recharge && Charge_count < 4){return;}
		      curgoalpoint_ = cur_goal_id.Taskarrived;
			  tc_->setTaskArrivedID(curgoalpoint_);
			  if(!cur_goal_id.Request) {return;}
			  if(PathDone != "" && cur_goal_id.Request){ PathDone += ("," + cur_goal_id.Taskarrived);}
			  if (cur_goal_id.Taskarrived != StopPoint_){return;}
			  Json::Value new_item,new_item_1,new_item_2;
			  std::string returnStr;
			  new_item_1["ErrorCode"] = 0;
			  new_item_1["Resultdata"] = StopPoint_;
			  Json::FastWriter writer;
			  std::string ResultStr = writer.write(new_item_1);
			  new_item_2["StopID"] = curgoalpoint_;
			  new_item_2["cor_x"] = _InspectInfo.PosX;
			  new_item_2["cor_y"] = _InspectInfo.PosY;
			  new_item_2["cor_angle"] = _InspectInfo.Angle;
			  std::string ParamStr = writer.write(new_item_2);
			  new_item["Token"] = "";
			  new_item["Topic"] = "CarArrivedPoint";
			  boost::uuids::uuid uuid = boost::uuids::random_generator()();
			  std::string struuid = boost::lexical_cast<std::string>(uuid);
			  new_item["SessionId"] = struuid;
			  time_t t;  //秒时间
			  t = time(NULL); //获取目前秒时间
			  new_item["Time"] = (Json::Int64)t;
			  new_item["Param"] = ParamStr;
			  new_item["Result"] = ResultStr;
			  returnStr = writer.write(new_item);
			  echo_server.send(m_msg_info.client_hdl, returnStr, (m_msg_info.message)->get_opcode());
						  std::cout<< "task_arrived------"  << (m_msg_info.client_hdl).lock().get()
																<< " and message: " << (m_msg_info.message)->get_payload()
																<< std::endl;
    	  }catch(std::exception& error){
			  ROS_LOG(ros::console::levels::Level::Error,"task_arrived massage error %s",error.what());
		  }
      }

      websocketpp::connection_hdl _hdl;
      CallbackPtr callback;

      void comm_parse(msg_info* p_msg_info){
    	  msg_info* pmsgInfo = p_msg_info;
      }

      void command_filter()
      {
    	  msg_info* p_msg_info;
    	  bool bRet = false;
    	  ros::NodeHandle n;
    	  ros::Rate r(controller_frequency_*2); //controller_frequency_

    	  while(n.ok())
    	  {
    		  r.sleep();
    		  bRet = que_msg.pop(p_msg_info);
    		  boost::thread* command_parse_thread = NULL;
    		  try{
				  if(bRet)
				  {
					  std::cout << "_hdl.lock().get() 0--" << p_msg_info->client_hdl.lock().get() << std::endl;
					  std::cout <<"-------queen out  message:--------------------"<< (p_msg_info->message)->get_payload() << std::endl;
					  if(p_msg_info->client_hdl.lock().get() > 0)
					  {
						  std::cout << "_hdl.lock().get() 1--" << p_msg_info->client_hdl.lock().get() << std::endl;
						  _hdl = p_msg_info->client_hdl;

//						  CallbackType callback_fun = boost::bind(&MoveBase::comm_parse,this,(void*)p_msg_info);
//						  boost::thread command_parse_thread = boost::thread(callback_fun);
//						  r.sleep();
//						  command_parse_thread.join();

						  boost::thread command_parse_thread = boost::thread(boost::bind(&MoveBase::command_parse,this,p_msg_info));
						  r.sleep();
						  command_parse_thread.join();

//						  callback = (CallbackPtr)(&MoveBase::comm_parse);
//						  boost::thread command_parse_thread = boost::thread(callback);
//						  r.sleep();
//						  command_parse_thread.join();
					  }
					  delete p_msg_info;
					  //command_parse_thread->detach();
					  //delete command_parse_thread;
					  //command_parse_thread.detach();
				  }
    		  }catch (websocketpp::exception const & e) {
    			  std::cout << e.what() << std::endl;
    		  }catch(std::exception& error){
    			  ROS_LOG(ros::console::levels::Level::Error,"command_filter error %s",error.what());
    			  std::cout << error.what() << std::endl;
    		  }
    		  catch (...) {
    			  std::cout << "other exception" << std::endl;
    		  }
    	  }
	  }

      int websocket_loop(){
		  try {
			  // Set logging settings
			  echo_server.set_access_channels(websocketpp::log::alevel::all);
			  echo_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

			  // Initialize Asio
			  echo_server.init_asio();

			  // Register our message handler
			  echo_server.set_message_handler(bind(&WebsocketServer::on_message, &echo_server, _1, _2));

			  // Listen on port 9000
			  echo_server.listen(webport_);

			  // Start the server accept loop
			  echo_server.start_accept();

			  // Start the ASIO io_service run loop
			  echo_server.run();

		  } catch (websocketpp::exception const & e) {
			  std::cout << e.what() << std::endl;
		  } catch (...) {
			  std::cout << "other exception" << std::endl;
		  }
		  return 0;
      }


      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
      boost::mutex tc_set_mutex_;
  };
};

#ifdef __cplusplus
}
#endif

#endif

