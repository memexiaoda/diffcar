#include "model_trx.h"
#include <math.h>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

namespace Chassis_Driver{
	bool Model_Trx::Init_model()
	{
		rs485.Init_model(dev_serial_rs485.c_str(),port_speed);

		filter[0].can_id = 0x181;
		filter[0].can_mask = CAN_SFF_MASK;
		filter[1].can_id = 0x281;
		filter[1].can_mask = CAN_SFF_MASK;
		filter[2].can_id = 0x381;
		filter[2].can_mask = CAN_SFF_MASK;
		filter[3].can_id = 0x481;
		filter[3].can_mask = CAN_SFF_MASK;

		imu_can.Init_model(1,1000000,filter,4);

		feedback_msg_received_filter = boost::bind(&Model_Trx::feedback_msg_Received, this);
		feedback_msg_received_thread = new boost::thread(feedback_msg_received_filter);

		control_msg_send_filter = boost::bind(&Model_Trx::control_msg_send, this);
		control_msg_send_thread = new boost::thread(control_msg_send_filter);

		control_msg_wrapper_filter = boost::bind(&Model_Trx::control_msg_wrapper, this);
		control_msg_wrapper_thread = new boost::thread(control_msg_wrapper_filter);

		_can_received_filter = boost::bind(&Model_Trx::Can_Received, this);
		_can_received_thread = new boost::thread(_can_received_filter);

		_can_send_filter = boost::bind(&Model_Trx::Can_Send, this);
		_can_send_thread = new boost::thread(_can_send_filter);

		feedback_msg_analysis_filter = boost::bind(&Model_Trx::feedback_msg_analysis, this);
		feedback_msg_analysis_thread = new boost::thread(feedback_msg_analysis_filter);

		//_imu_pub_filter = boost::bind(&Model_Trx::Can_Received, this);
		target_linear_vel = 0.0;
		target_angular_vel = 0.0;
		control_style = "manual";

		//feedback_msg_received_thread->join();
		control_msg_wrapper_thread->join();
		control_msg_send_thread->join();
		feedback_msg_analysis_thread->join();

		_can_received_thread->join();
		_can_send_thread->join();
		
		obstacle_dis_laser_low = 6.0;
		obstacle_dis_laser_high = 6.0;

		for(int i=0; i<5; i++){
			ultrasonic_dis[i] = 0.0;
		}

		_InspectInfo.StartPoint = _DockPile;
		_InspectInfo.EndPoint = _DockPoint;
		_InspectInfo.LineID = std::string(_DockPile + _DockPoint);
		_batteryinfo.ExternalPowerSupply = false;
		ultrasonic_dis_front = -0.01;
		return true;
	}

	__u8 Model_Trx::crc_check(void* start_buffer,size_t size){
		__u8 crc_sum = 0;
		for(int i=0 ; i < size ; i++){
			crc_sum += ((__u8*)start_buffer)[i];
		}
		return crc_sum;
	}

	void Model_Trx::controlstyleCallback(const std_msgs::String& controlstyle){
		control_style = controlstyle.data;
		ROS_INFO("controlstyle.data = %s",controlstyle.data.c_str());
	}

	void Model_Trx::powerstopCallback(const std_msgs::Bool& powerstop)
	{
		power_stop_ = powerstop.data;
	}

	void Model_Trx::taskgoalCallback(const std_msgs::String& taskgoal)
	{
		taskgoal_ = taskgoal.data;
	}

	void Model_Trx::ManualctrolCallback(const carbot_msgs::Manualcmd& manualcmd){

		commond_linear_vel = manualcmd.vel_value.linear.x;
		commond_angular_vel = manualcmd.vel_value.angular.z;

		if(manualcmd.action_cmd == "forward"){
			target_linear_vel = commond_linear_vel;
			target_angular_vel = 0.0;//commond_angular_vel;
		}else if(manualcmd.action_cmd == "backward"){
			target_linear_vel = -commond_linear_vel;
			target_angular_vel = 0.0;//commond_angular_vel;
		}else if(manualcmd.action_cmd == "turnleft"){
			target_linear_vel = 0.0;//-commond_linear_vel;
			target_angular_vel = commond_angular_vel;
		}else if(manualcmd.action_cmd == "turnright"){
			target_linear_vel = 0.0;//-commond_linear_vel;
			target_angular_vel = -commond_angular_vel;
		}else{
			target_linear_vel = 0.0;
			target_angular_vel = 0.0;//commond_angular_vel;
		}
	}

	void Model_Trx::led_command_Callback(const carbot_msgs::LedMsg& led_msg)
	{
		static int greencount = 0;
		static int redcount = 0;
		static int yellowcount = 0;

		if(led_msg.colors == "red"){
			if(!led_msg.twinkle){
				_equipmentInfo.LampColor = 1;
				control_msg.LED_CW = 1;
			}else{
				_equipmentInfo.LampColor = 4;
				control_msg.LED_CW = 4;
			}
		}else if(led_msg.colors == "green"){
			if(!led_msg.twinkle){
				_equipmentInfo.LampColor = 2;
				control_msg.LED_CW = 2;
			}else{
				_equipmentInfo.LampColor = 5;
				control_msg.LED_CW = 5;
			}
		}else if(led_msg.colors == "yellow"){
			if(!led_msg.twinkle){
				_equipmentInfo.LampColor = 3;
				control_msg.LED_CW = 3;
			}else{
				_equipmentInfo.LampColor = 6;
				control_msg.LED_CW = 6;
			}
		}else{
			_equipmentInfo.LampColor = 0;
			control_msg.LED_CW = 0;
		}
	}

	void Model_Trx::move_recvCallback(const carbot_msgs::InspectInfo& msg){
		_InspectInfo = msg;
		InspectInfo_pub.publish(_InspectInfo);
	}

	void Model_Trx::pose_attitude_recv(const std_msgs::String& pose_attitude)
	{
		planner_state = pose_attitude.data;
	}

	void Model_Trx::laser_state_recv(const std_msgs::String& laser_state)
	{
		high_laser_state = laser_state.data;
	}

	void Model_Trx::scanstopCallback(const sensor_msgs::LaserScanConstPtr& scanstop){
		  sensor_msgs::LaserScan laser_data;
		  laser_data = *scanstop;
		  laser_data.header.stamp = ros::Time::now();

		  int i = int(laser_data.ranges.size()/720.0 * (360 - LObstacle_low_sector_));
		  int j = int(laser_data.ranges.size()/360.0 * LObstacle_low_sector_);

		  double dis_average = 0.0;
		  double distance = 10.0;
		  int count = 0;
	      static double obstacle_dis_laser0_last[5]= {-0.01};
	      static int count_ca = 0;

		  for(int k=0; k<j;k++)
		  {
			  if(laser_data.ranges[k+i] < LObstacle_low_Slow_)
			  {
				  count += 1;
				  //distance += laser_data.ranges[k+i];
 				  distance = std::min(distance,(double)laser_data.ranges[k+i]);
			  }
		  }

		  if (count >= 2)
			  dis_average = (double)distance;//(double)distance/count;
		  else
			  dis_average = -0.01;

		  obstacle_dis_laser0_last[0] = obstacle_dis_laser0_last[1];
		  obstacle_dis_laser0_last[1] = obstacle_dis_laser0_last[2];
		  obstacle_dis_laser0_last[2] = obstacle_dis_laser0_last[3];
		  obstacle_dis_laser0_last[3] = obstacle_dis_laser0_last[4];
		  obstacle_dis_laser0_last[4] = dis_average;

		  if(obstacle_dis_laser0_last[0] > 0 ){
			  if(obstacle_dis_laser0_last[1] < 0 && obstacle_dis_laser0_last[2] < 0  && obstacle_dis_laser0_last[3] > 0) {
				  obstacle_dis_laser0_last[1] = obstacle_dis_laser0_last[2] = obstacle_dis_laser0_last[3];
			  }else{
				  if(obstacle_dis_laser0_last[1] < 0 && obstacle_dis_laser0_last[2] > 0){
					  obstacle_dis_laser0_last[1] = obstacle_dis_laser0_last[2];
				  }
			  }
		  }

		  if (dis_average >0){
			  obstacle_dis_laser0_last[0] = dis_average;
			  obstacle_dis_laser0_last[1] = dis_average;
			  obstacle_dis_laser0_last[2] = dis_average;
			  obstacle_dis_laser0_last[3] = dis_average;
			  obstacle_dis_laser0_last[4] = dis_average;
		  }

		  obstacle_dis_laser_low = obstacle_dis_laser0_last[0];
	      obstacle_data.y = obstacle_dis_laser_low;
	}

	void Model_Trx::laserCB(const sensor_msgs::LaserScan::ConstPtr& laser_scan_data){
		  sensor_msgs::LaserScan laser_data;
		  laser_data = *laser_scan_data;
		  laser_data.header.stamp = ros::Time::now();

		  int i = int(laser_data.ranges.size()/720.0 * (360 - LObstacle_front_sector_));
		  int j = int(laser_data.ranges.size()/360.0 * LObstacle_front_sector_);

		  double dis_average = 0.0;
		  double distance = 10.0;
		  int count = 0;

		  static double obstacle_dis_last[5]= {-0.01};
		  //static double obstacle_dis_last2= -0.01;
		  static int count_ca = 0;
		  for(int k=0; k<j; k++){
			  if(laser_data.ranges[k+i] < LObstacleSlow_){
				  count += 1;
				  //distance += laser_data.ranges[k+i];//data.ranges[k+i]
				  distance = std::min(distance,(double)laser_data.ranges[k+i]);
			  }
		  }

		  if (count >= 2)
			  dis_average = (double)distance;//(double)distance/count;
		  else
			  dis_average = -0.01;

		  //obstacle_dis = dis_average;
		  obstacle_dis_last[0] = obstacle_dis_last[1];
		  obstacle_dis_last[1] = obstacle_dis_last[2];
		  obstacle_dis_last[2] = obstacle_dis_last[3];
		  obstacle_dis_last[3] = obstacle_dis_last[4];
		  obstacle_dis_last[4] = dis_average;

		  if(obstacle_dis_last[0] > 0 ){
			  if(obstacle_dis_last[1] < 0 && obstacle_dis_last[2] < 0  && obstacle_dis_last[3] > 0){
				  obstacle_dis_last[1] = obstacle_dis_last[2] = obstacle_dis_last[3];
			  }else{
				  if(obstacle_dis_last[1] < 0 && obstacle_dis_last[2] > 0 )
				  {
					  obstacle_dis_last[1] = obstacle_dis_last[2];
				  }
			  }
		  }

		  if (dis_average >0)
		  {
			  obstacle_dis_last[0] = dis_average;
			  obstacle_dis_last[1] = dis_average;
			  obstacle_dis_last[2] = dis_average;
			  obstacle_dis_last[3] = dis_average;
			  obstacle_dis_last[4] = dis_average;
		  }

		  obstacle_dis_laser_high = obstacle_dis_last[0];
		  ROS_INFO_DELAYED_THROTTLE(2,"High obstacle_dis_laser_high = %.3f", obstacle_dis_laser_high);
		  obstacle_data.x = obstacle_dis_laser_high;
	}
    //底层激光雷达数据接收函数
	void Model_Trx::recvCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
	{
	  // Latch ring count
	  if (!ring_count_) {
	    // Check for PointCloud2 field 'ring'
	    bool found = false;
	    for (size_t i = 0; i < msg->fields.size(); i++) {
	      if (msg->fields[i].datatype == sensor_msgs::PointField::UINT16) {
	        if (msg->fields[i].name == "ring") {
	          found = true;
	          break;
	        }
	      }
	    }

	    if (!found) {
	      ROS_ERROR("VelodyneLaserScan: Field 'ring' of type 'UINT16' not present in PointCloud2");
	      return;
	    }
	    for (sensor_msgs::PointCloud2ConstIterator<uint16_t> it(*msg, "ring"); it != it.end(); ++it) {
	      const uint16_t ring = *it;
	      if (ring + 1 > ring_count_) {
	        ring_count_ = ring + 1;
	      }
	    }

	    if (ring_count_) {
	      ROS_INFO("VelodyneLaserScan: Latched ring count of %u", ring_count_);
	    } else {
	      ROS_ERROR("VelodyneLaserScan: Field 'ring' of type 'UINT16' not present in PointCloud2");
	      return;
	    }
	  }
	  // Select ring to use
	  uint16_t ring;
	  //雷达线号（0-15）(0-最下线，15最上线）
	  int cfg_ring = 0;

	  if ((cfg_ring < 0) || (cfg_ring >= ring_count_)) {
	    // Default to ring closest to being level for each known sensor
	    if (ring_count_ > 32) {
	      ring = 57; // HDL-64E
	    } else if (ring_count_ > 16) {
	      ring = 23; // HDL-32E
	    } else {
	      ring = 8; // VLP-16
	    }
	  } else {
	    ring = cfg_ring;
	  }
	  ROS_INFO_ONCE("VelodyneLaserScan: Extracting ring %u", ring);
	  // Load structure of PointCloud2
	  int offset_x = -1;
	  int offset_y = -1;
	  int offset_z = -1;
	  int offset_i = -1;
	  int offset_r = -1;
	  for (size_t i = 0; i < msg->fields.size(); i++) {
	    if (msg->fields[i].datatype == sensor_msgs::PointField::FLOAT32) {
	      if (msg->fields[i].name == "x") {
	        offset_x = msg->fields[i].offset;
	      } else if (msg->fields[i].name == "y") {
	        offset_y = msg->fields[i].offset;
	      } else if (msg->fields[i].name == "z") {
	        offset_z = msg->fields[i].offset;
	      } else if (msg->fields[i].name == "intensity") {
	        offset_i = msg->fields[i].offset;
	      }
	    } else if (msg->fields[i].datatype == sensor_msgs::PointField::UINT16) {
	      if (msg->fields[i].name == "ring") {
	        offset_r = msg->fields[i].offset;
	      }
	    }
	  }

	  // Construct LaserScan message
	  if ((offset_x >= 0) && (offset_y >= 0) && (offset_r >= 0)) {
	    const float RESOLUTION = fabsf(0.007);
	    const size_t SIZE = 2.0 * M_PI / RESOLUTION;
	    sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());
	    scan->header = msg->header;
	    scan->angle_increment = RESOLUTION;
	    scan->angle_min = -M_PI;
	    scan->angle_max = M_PI;
	    scan->range_min = 0.1;
	    scan->range_max = 100.0;
	    scan->time_increment = 0.0;
	    scan->ranges.resize(SIZE, INFINITY);
	    if ((offset_x == 0) && (offset_y == 4) && (offset_z == 8) && (offset_i == 16) && (offset_r == 20)) {
	      scan->intensities.resize(SIZE);
	      for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
	        const uint16_t r = *((const uint16_t*)(&it[5])); // ring
	        if (r == ring) {
	          const float x = it[0]; // x
	          const float y = it[1]; // y
	          const float i = it[4]; // intensity
	          const int bin = (atan2f(y, x) + (float)M_PI) / RESOLUTION;
	          if ((bin >= 0) && (bin < (int)SIZE)) {
	            scan->ranges[bin] = sqrtf(x * x + y * y);
	            scan->intensities[bin] = i;
	          }
	        }
	      }
	    } else {
	      ROS_WARN_ONCE("VelodyneLaserScan: PointCloud2 fields in unexpected order. Using slower generic method.");
	      if (offset_i >= 0) {
	        scan->intensities.resize(SIZE);
	        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
	        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
	        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
	        sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
	        for ( ; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r, ++iter_i) {
	          const uint16_t r = *iter_r; // ring
	          if (r == ring) {
	            const float x = *iter_x; // x
	            const float y = *iter_y; // y
	            const float i = *iter_i; // intensity
	            const int bin = (atan2f(y, x) + (float)M_PI) / RESOLUTION;
	            if ((bin >= 0) && (bin < (int)SIZE)) {
	              scan->ranges[bin] = sqrtf(x * x + y * y);
	              scan->intensities[bin] = i;
	            }
	          }
	        }
	      } else {
	        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
	        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
	        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
	        for ( ; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r) {
	          const uint16_t r = *iter_r; // ring
	          if (r == ring) {
	            const float x = *iter_x; // x
	            const float y = *iter_y; // y
	            const int bin = (atan2f(y, x) + (float)M_PI) / RESOLUTION;
	            if ((bin >= 0) && (bin < (int)SIZE)) {
	              scan->ranges[bin] = sqrtf(x * x + y * y);
	            }
	          }
	        }
	      }
	    }
	    pub_scan.publish(scan);
	  } else {
	    ROS_ERROR("VelodyneLaserScan: PointCloud2 missing one or more required fields! (x,y,ring)");
	  }
	}
	void Model_Trx::velCallback(const geometry_msgs::Twist& cmdvel)
	{
		//ROS_INFO("Model_Kinco::velCallback");
		if(control_style == "auto")
		{
			target_linear_vel = cmdvel.linear.x;
			target_angular_vel = cmdvel.angular.z;
		}
	}

	void Model_Trx::velCallbackmanual(const geometry_msgs::Twist& cmd_vel_manual){
		//ROS_INFO("Model_Trx::velCallbackmanual");
		control_style = "manual";
		target_linear_vel = cmd_vel_manual.linear.x;
		target_angular_vel = cmd_vel_manual.angular.z;
	}

	void Model_Trx::calc_speed()
	{
		if(_cmdvel.angular.z > 0.0)
		{
			left_speed = _cmdvel.linear.x - fabs(tan(_cmdvel.angular.z)*base_wide*0.5);
			right_speed = _cmdvel.linear.x + fabs(tan(_cmdvel.angular.z)*base_wide*0.5);
		}
		else if(_cmdvel.angular.z == 0.0)
		{
			left_speed = _cmdvel.linear.x;
			right_speed = _cmdvel.linear.x;
		}
		else
		{
			left_speed = _cmdvel.linear.x + fabs(tan(_cmdvel.angular.z)*base_wide*0.5);
			right_speed = _cmdvel.linear.x - fabs(tan(_cmdvel.angular.z)*base_wide*0.5);
		}
		//ROS_INFO("left_speed=%.3f,right_speed=%.3f", left_speed, right_speed);
	}

	void  Model_Trx::feedback_msg_analysis()
	{
		double elapsed_sec;
		ros::Time now = ros::Time::now();
		ros::Time then = ros::Time::now();
		while(nh_.ok()){
			ros::spinOnce();
			rate_ultra.sleep();
			static int count = 0;
			/*
			 * 车体设备信息 EquipmentInfo
			 */
			_equipmentInfo.Obt1_Dis = _feedback_msg.Obt1_Dis;
			_equipmentInfo.Obt2_Dis = _feedback_msg.Obt2_Dis;
			_equipmentInfo.Obt3_Dis = _feedback_msg.Obt3_Dis;
			_equipmentInfo.FP1_Status = _feedback_msg.FP1_Status;
			_equipmentInfo.FP2_Status = _feedback_msg.FP2_Status;
			_equipmentInfo.FP1_Dis = _feedback_msg.FP1_Dis;
			_equipmentInfo.FP2_Dis = _feedback_msg.FP2_Dis;

			//_equipmentInfo.FC_Alarm = (bool)_feedback_msg.FC_Alarm;//前碰撞信息
			//_equipmentInfo.OA_Alarm = (bool)_feedback_msg.OA_Alarm;//前避障信息
			_equipmentInfo.ES_Alarm = (bool)_feedback_msg.ES_Alarm;//急停信息
			//_equipmentInfo.FP_Alarm = (bool)_feedback_msg.FP_Alarm;//防跌落信息
			_equipmentInfo.OBT_Status = _feedback_msg.OBT_Status;//超声波在线状态
			//车轮在线状态
			_equipmentInfo.LFW_Status = _feedback_msg.LFW_Status;
			_equipmentInfo.LBW_Status = _feedback_msg.LBW_Status;
			_equipmentInfo.RFW_Status = _feedback_msg.RFW_Status;
			_equipmentInfo.RBW_Status = _feedback_msg.RBW_Status;
			_equipmentInfo.ObstacleWarning = ObstacleWarning;
			//雷达通信状态
			if(laser_scan_sub == NULL){
				_equipmentInfo.Laser_Status = 1;
			}else{
				_equipmentInfo.Laser_Status = 0;
			}

			_equipmentInfo.TEMP_Chassis = _feedback_msg.TEMP_Chassis;//车体温度
			_equipmentInfo.RunningSpeed = _cmdvel_buff.linear.x;//运行速度信息

			//BMS信息
			_batteryinfo.Battery_Volt = _feedback_msg.Battery_Volt;
			_batteryinfo.Battery_Current = _feedback_msg.Battery_Current;
			_batteryinfo.Battery_Power = _feedback_msg.Battery_Power;
			_batteryinfo.Battery_Temp = _feedback_msg.Battery_Temp;
			_batteryinfo.BC_Status = _feedback_msg.BC_Status;
			_batteryinfo.ExternalPowerSupply;
			//
			_driverInfo.LFMotor_Status = _feedback_msg.LFMotor_Status;
			_driverInfo.RFMotor_Status = _feedback_msg.RFMotor_Status;
			_driverInfo.LBMotor_Status = _feedback_msg.LBMotor_Status;
			_driverInfo.RBMotor_Status = _feedback_msg.RBMotor_Status;
			_driverInfo.LFMotor_Error = _feedback_msg.LFMotor_Error;
			_driverInfo.RFMotor_Error = _feedback_msg.RFMotor_Error;
			_driverInfo.LBMotor_Error = _feedback_msg.LBMotor_Error;
			_driverInfo.RBMotor_Error = _feedback_msg.RBMotor_Error;

			equipmentInfo_pub.publish(_equipmentInfo);
			driverInfo_pub.publish(_driverInfo);
			batteryinfo_pub.publish(_batteryinfo);
		}
	}

	void Model_Trx::control_msg_wrapper()
	{
		ROS_INFO("control_msg_wrapper");
		obstacle_dis_laser_low = -0.1;
		obstacle_dis_laser_high = -0.1;
		ultrasonic_dis_front = -0.1;
		int16_t high_wd=0, low_wd=0, ultra_wd=0, fp_wd=0;
		geometry_msgs::Vector3 stop_state;
		while(nh_.ok())
		{
			ros::spinOnce();
			rate_send.sleep();
			//high_wd=0, low_wd=0, ultra_wd=0, fp_wd =0;
			//_equipmentInfo.ObstacleWarning = 0;
			//状态 stop = 1; slow = 2; normal = 0;
			//  0度 上层激光雷达避障状态

			int high_laser_stop_state;
			if(obstacle_dis_laser_high <= LObstacleSlow_ && obstacle_dis_laser_high > LObstacleStop_){
				high_laser_stop_state = 2;//slow
			}else if(obstacle_dis_laser_high <= LObstacleStop_ && obstacle_dis_laser_high > 0.0){
				high_laser_stop_state = 1;//stop
			}else{
				high_laser_stop_state = 0;//normal
			}

			//-15度 底层激光雷达避障状态
			int low_laser_stop_state;
			if(obstacle_dis_laser_low <= LObstacle_low_Stop_ && obstacle_dis_laser_low > 0.0){
				low_laser_stop_state = 1;//stop
			}else if(obstacle_dis_laser_low <= LObstacle_low_Slow_ && obstacle_dis_laser_low > LObstacle_low_Stop_){
				low_laser_stop_state = 2;//slow
			}else{
				low_laser_stop_state = 0;//normal
			}

			// 前端超声波避障状态
			int ultrasonic_stop_state; //= _feedback_msg.OA_Alarm;
			uint16_t min_obt_dis;

			_feedback_msg.Obt1_Dis =  _feedback_msg.Obt1_Dis > 25 ? _feedback_msg.Obt1_Dis : 500;
			_feedback_msg.Obt2_Dis =  _feedback_msg.Obt2_Dis > 25 ? _feedback_msg.Obt2_Dis : 500;
			_feedback_msg.Obt3_Dis =  _feedback_msg.Obt3_Dis > 25 ? _feedback_msg.Obt3_Dis : 500;

			min_obt_dis = std::min( _feedback_msg.Obt1_Dis, _feedback_msg.Obt2_Dis);
			min_obt_dis = std::min(min_obt_dis, _feedback_msg.Obt3_Dis);
			ultrasonic_dis_front = min_obt_dis;
			ultrasonic_dis_front /= 100.0;

//			static double ultrasonic_dis_front_last[2]= {-0.01};
//			ultrasonic_dis_front_last[0] = ultrasonic_dis_front_last[1];
//			ultrasonic_dis_front_last[1] = ultrasonic_dis_front;
//			ultrasonic_dis_front = max_obt;
//			double ultrasonic_dis_front_sum;
//
//			for(int i=0 ; i < 2; i++){
//				ultrasonic_dis_front_sum += ultrasonic_dis_front_last[i];
//			}
//			ultrasonic_dis_front = ultrasonic_dis_front_sum / 2;

			obstacle_data.z = ultrasonic_dis_front;

			if(ultrasonic_dis_front <= LObstacle_Ultrasonic_Stop_ && ultrasonic_dis_front > 0.25){
				ultrasonic_stop_state = 1;//stop
			}else if(ultrasonic_dis_front <= LObstacle_Ultrasonic_Slow_ && ultrasonic_dis_front > LObstacle_Ultrasonic_Stop_){
				ultrasonic_stop_state = 2;//slow
			}else{
				ultrasonic_stop_state = 0;//normal
			}

			stop_state.x = high_laser_stop_state;
			stop_state.y = low_laser_stop_state;
			stop_state.z = ultrasonic_stop_state;

			if(high_laser_stop_state == 1){
				high_wd = 1;
			}else{
				high_wd = 0;
			}
			if(low_laser_stop_state == 1){
				low_wd = 2;
			}else{
				low_wd = 0;
			}

			if(ultrasonic_stop_state == 1){
				ultra_wd = 4;
				_equipmentInfo.OA_Alarm = 1;//前避障信息
			}else{
				ultra_wd = 0;
				_equipmentInfo.OA_Alarm = 0;//前避障信息
			}
                         
//			if((obstacle_dis_laser_low < LowObstacle_Limit_ && obstacle_dis_laser_low > 0
//							&& obstacle_dis_laser_high < 0) && LowObstacle_enable_ == true){
//				low_wd = 16;
//						//_equipmentInfo.ObstacleWarning = high_wd | low_wd | ultra_wd | fp_wd;
//			}else{
//				low_wd = 0;
//			}
			ObstacleWarning = high_wd + low_wd + ultra_wd +fp_wd;

			/*
			 * 绕行延迟计时函数
			 */
			static ros::Time delay_motion_start_time = ros::Time::now();
			static ros::Time delay_motion_end_time = ros::Time::now();
			static bool bestart_record = false;
			carbot_msgs::Delaytimerecorder delay_timer;
			double delay_motion_time;

			if(control_style == "manual")
			{
				ROS_ERROR_DELAYED_THROTTLE(2,"control_style == manual");
				if(/*high_laser_stop_state == 1 || low_laser_stop_state == 1 || ultrasonic_stop_state == 1|| */ObstacleWarning != 0 || _equipmentInfo.FP_Alarm != 0)
				{
					//停车
					if(target_linear_vel > 0){
						target_linear_vel = 0.0;
					}
					target_angular_vel = 0.0;
					ROS_WARN_DELAYED_THROTTLE(2.0,"manual chassis obstacle stop");
					//ROS_LOG_DELAYED_THROTTLE(1.0,ros::console::levels::Level::Warn, "chassis obstacle stop");
					//障碍物速度缓冲区,保证车体的运动流畅性
					if (target_linear_vel < _cmdvel_buff.linear.x ){
						_cmdvel_buff.linear.x = std::max(target_linear_vel,float (_cmdvel_buff.linear.x - acc_lim_xDC_chassis_));
					}else{
						_cmdvel_buff.linear.x = std::min(target_linear_vel,float(_cmdvel_buff.linear.x + acc_lim_xAD_chassis_));
					}
					if (target_angular_vel < _cmdvel_buff.angular.z){
						_cmdvel_buff.angular.z = std::max(target_angular_vel,float(_cmdvel_buff.angular.z - acc_lim_theta_chassis_));//#0.02
					}else{
						_cmdvel_buff.angular.z = std::min(target_angular_vel,float(_cmdvel_buff.angular.z + acc_lim_theta_chassis_));
					}
				}else if(high_laser_stop_state == 2 && low_laser_stop_state == 2 && ultrasonic_stop_state == 2){
					//减速
					ROS_WARN_DELAYED_THROTTLE(2.0,"manual chassis obstacle slow");
					if(fabs(target_linear_vel) > 0.1){
						target_linear_vel = 0.1 * sign(target_linear_vel);
					}
					//障碍物速度缓冲区,保证车体的运动流畅性
					if (target_linear_vel < _cmdvel_buff.linear.x ){
						_cmdvel_buff.linear.x = std::max(target_linear_vel,float (_cmdvel_buff.linear.x - acc_lim_xDC_chassis_));//#0.02
					}else{
						_cmdvel_buff.linear.x = std::min(target_linear_vel,float(_cmdvel_buff.linear.x + acc_lim_xAD_chassis_));
					}
					if (target_angular_vel < _cmdvel_buff.angular.z ){
						_cmdvel_buff.angular.z = std::max(target_angular_vel,float(_cmdvel_buff.angular.z - acc_lim_theta_chassis_));//#0.02
					}else{
						_cmdvel_buff.angular.z = std::min(target_angular_vel,float(_cmdvel_buff.angular.z + acc_lim_theta_chassis_));
					}
				}else{
					//无障碍物区
					//速度缓冲区,保证车体的运动流畅性
					if (target_linear_vel < _cmdvel_buff.linear.x){
						_cmdvel_buff.linear.x = std::max(target_linear_vel,float (_cmdvel_buff.linear.x - acc_lim_xDC_chassis_));
					}else{
						_cmdvel_buff.linear.x = std::min(target_linear_vel,float(_cmdvel_buff.linear.x + acc_lim_xAD_chassis_));
					}
					if (target_angular_vel < _cmdvel_buff.angular.z){
						_cmdvel_buff.angular.z = std::max(target_angular_vel,float(_cmdvel_buff.angular.z - acc_lim_theta_chassis_));//#0.02
					}else{
						_cmdvel_buff.angular.z = std::min(target_angular_vel,float(_cmdvel_buff.angular.z + acc_lim_theta_chassis_));
					}
				}
			}else
			{
				/*
				 * 雷达及超声波报警
				 */
				ROS_ERROR_DELAYED_THROTTLE(2,"Model_Trx _carinfo.EndPoint = %s", _InspectInfo.EndPoint.c_str());

				if( _InspectInfo.LineID == (_DockPile + "-" + _DockPoint))
				{
					ROS_ERROR_DELAYED_THROTTLE(2,"obstacle_dis_laser_low N ultrasonic_dis_front is none");

					//速度缓冲区,保证车体的运动流畅性
					if(target_linear_vel < _cmdvel_buff.linear.x ){
						_cmdvel_buff.linear.x = std::max(target_linear_vel,float (_cmdvel_buff.linear.x - acc_lim_xDC_chassis_));//#0.02
					}else{
						_cmdvel_buff.linear.x = std::min(target_linear_vel,float(_cmdvel_buff.linear.x + acc_lim_xAD_chassis_));
					}
					if(target_angular_vel < _cmdvel_buff.angular.z){
						_cmdvel_buff.angular.z = std::max(target_angular_vel,float(_cmdvel_buff.angular.z - acc_lim_theta_chassis_));//#0.02
					}else{
						_cmdvel_buff.angular.z = std::min(target_angular_vel,float(_cmdvel_buff.angular.z + acc_lim_theta_chassis_));
					}
				}
				else
				{
					if(high_laser_stop_state == 1 || low_laser_stop_state == 1 || ultrasonic_stop_state == 1)
					{
						//ROS_INFO("obstacle_dis_laser_low = %f", obstacle_dis_laser_low);
						//停车
						if(planner_state == "turn"){
							target_linear_vel = 0.0;
						}else{
							if(target_linear_vel > 0){
								target_linear_vel = 0.0;
							}
							target_angular_vel = 0.0;
						}
						ROS_WARN_DELAYED_THROTTLE(2.0,"chassis obstacle stop");
						//障碍物速度缓冲区,保证车体的运动流畅性
						if (target_linear_vel < _cmdvel_buff.linear.x ){
							_cmdvel_buff.linear.x = std::max(target_linear_vel,float (_cmdvel_buff.linear.x - acc_lim_xDC_chassis_));
						}else{
							_cmdvel_buff.linear.x = std::min(target_linear_vel,float(_cmdvel_buff.linear.x + acc_lim_xAD_chassis_));
						}
						if (target_angular_vel < _cmdvel_buff.angular.z){
							_cmdvel_buff.angular.z = std::max(target_angular_vel,float(_cmdvel_buff.angular.z - acc_lim_theta_chassis_));//#0.02
						}else{
							_cmdvel_buff.angular.z = std::min(target_angular_vel,float(_cmdvel_buff.angular.z + acc_lim_theta_chassis_));
						}
					}else if(high_laser_stop_state == 2 && low_laser_stop_state == 2 && ultrasonic_stop_state == 2){
						//减速
						if(planner_state == "turn"){
							target_linear_vel = 0.0;
						}else{
							if(fabs(target_linear_vel) > 0.1){
								target_linear_vel = 0.1 * sign(target_linear_vel);
							}
						}
						ROS_WARN_DELAYED_THROTTLE(2.0,"chassis obstacle slow");
						//障碍物速度缓冲区,保证车体的运动流畅性
						if (target_linear_vel < _cmdvel_buff.linear.x ){
							_cmdvel_buff.linear.x = std::max(target_linear_vel,float (_cmdvel_buff.linear.x - acc_lim_xDC_chassis_));//#0.02
						}else{
							_cmdvel_buff.linear.x = std::min(target_linear_vel,float(_cmdvel_buff.linear.x + acc_lim_xAD_chassis_));
						}
						if (target_angular_vel < _cmdvel_buff.angular.z ){
							_cmdvel_buff.angular.z = std::max(target_angular_vel,float(_cmdvel_buff.angular.z - acc_lim_theta_chassis_));//#0.02
						}else{
							_cmdvel_buff.angular.z = std::min(target_angular_vel,float(_cmdvel_buff.angular.z + acc_lim_theta_chassis_));
						}
					}else{
						//无障碍物区
						if(planner_state == "turn"){
							target_linear_vel = 0.0;
						}
						ROS_WARN_DELAYED_THROTTLE(2.0,"auto move");
						//速度缓冲区,保证车体的运动流畅性
						if (target_linear_vel < _cmdvel_buff.linear.x){
							_cmdvel_buff.linear.x = std::max(target_linear_vel,float (_cmdvel_buff.linear.x - acc_lim_xDC_chassis_));
						}else{
							_cmdvel_buff.linear.x = std::min(target_linear_vel,float(_cmdvel_buff.linear.x + acc_lim_xAD_chassis_));
						}
						if (target_angular_vel < _cmdvel_buff.angular.z){
							_cmdvel_buff.angular.z = std::max(target_angular_vel,float(_cmdvel_buff.angular.z - acc_lim_theta_chassis_));//#0.02
						}else{
							_cmdvel_buff.angular.z = std::min(target_angular_vel,float(_cmdvel_buff.angular.z + acc_lim_theta_chassis_));
						}
					}
				}

				//如果有障碍物开始计时
				if(_equipmentInfo.ObstacleWarning != 0 /* && control_style == "auto"|| _equipmentInfo.FP_Alarm != 0*/)
				{
					if(!bestart_record)
					{
						bestart_record = true;
						delay_motion_start_time = ros::Time::now();
					}
					carbot_msgs::Delayaction srv;
					static bool call_succed = false;
					delay_motion_end_time = ros::Time::now();
					delay_motion_time = delay_motion_end_time.toSec() - delay_motion_start_time.toSec();
					delay_timer.Delaytimerecoder = (int)delay_motion_time;
					if(delay_motion_time > event_limit_time && !call_succed)
					{
						srv.request.Overtime = true;
						delay_timer.Beovertime = srv.request.Overtime;
						srv.request.Arrived_goal = _InspectInfo.CurrentPoint;
						srv.request.Task_goal = _InspectInfo.EndPoint;
						srv.request.Current_lineID = _InspectInfo.LineID;
						srv.request.Limit_time = event_limit_time;
						call_succed = client_delay_motion.call(srv);
						if (call_succed)
						{
							ROS_WARN("delay Move_state: %u", srv.response.Delay_motion_state);
							delay_motion_start_time = ros::Time::now();
							delay_motion_end_time = ros::Time::now();
							delay_motion_time = 0;
							delay_timer.Beovertime = false;
							call_succed = false;
							bestart_record = false;
							delay_motion_time_pub.publish(delay_timer);
							ROS_INFO_DELAYED_THROTTLE(2.0,"obstacle stop delay_motion done");

						}else{
							ROS_WARN_DELAYED_THROTTLE(2.0,"Failed to call service delay_motion");
						}
					}
				}else{
					delay_motion_start_time = ros::Time::now();
					delay_motion_end_time = ros::Time::now();
				}
			}

			//ROS_INFO("_cmdvel_buff.linear.x=%.3f,_cmdvel_buff.angular.z=%.3f",_cmdvel_buff.linear.x,_cmdvel_buff.angular.z);
			/////////////////
			////前碰撞停车
			/////////////////
			static int FC_Alarm_State = 0;
			static ros::Time start_time = ros::Time::now();
			static ros::Time end_time = ros::Time::now();
			double delay_time;
			if(_feedback_msg.FC_Alarm == 1){
				delay_time = end_time.toSec() - start_time.toSec();
				if(FC_Alarm_State == 0){
					start_time = ros::Time::now();
					end_time = ros::Time::now();
					FC_Alarm_State = 1;
				}else{
					end_time = ros::Time::now();
				}
				//停车命令
				//电机抱闸
				target_linear_vel = 0.0;
				target_angular_vel = 0.0;
				_cmdvel_buff.linear.x = target_linear_vel;
				_cmdvel_buff.angular.z = target_angular_vel;
				if(control_style == "auto"){
					std_msgs::String ctrstyle;
					control_style = "manual";
					ctrstyle.data = control_style;
					control_style_pub.publish(ctrstyle);
				}
			}else{
				delay_time = end_time.toSec() - start_time.toSec();
				if(FC_Alarm_State == 0){
					start_time = ros::Time::now();
					end_time = ros::Time::now();					
				}else{
					if(delay_time > 10){
						FC_Alarm_State = 0;
						start_time = ros::Time::now();
						end_time = ros::Time::now();
					}else{
						end_time = ros::Time::now();
					}
				}
			}
			_equipmentInfo.FC_Alarm = (bool)FC_Alarm_State;//前碰撞信息
			/////////////////
			////车体过流应急
			/////////////////
			if(std::abs(_feedback_msg.LF_Torque) > 1600 || std::abs(_feedback_msg.RF_Torque) > 1600 || std::abs(_feedback_msg.LB_Torque) > 1600 || std::abs(_feedback_msg.RB_Torque) > 1600)
			{
				//停车命令
				//电机抱闸
				target_linear_vel = 0.0;
				target_angular_vel = 0.0;
				_cmdvel_buff.linear.x = target_linear_vel;
				_cmdvel_buff.angular.z = target_angular_vel;
				//if(control_style == "auto"){
				//	std_msgs::String ctrstyle;
				//	control_style = "manual";
				//	ctrstyle.data = control_style;
				//	control_style_pub.publish(ctrstyle);
				//}
			}

			if(_feedback_msg.LFMotor_Error != "0000" || _feedback_msg.LBMotor_Error != "0000" || _feedback_msg.RFMotor_Error != "0000" || _feedback_msg.RBMotor_Error != "0000"){
				//停车命令
				//电机抱闸
				target_linear_vel = 0.0;
				target_angular_vel = 0.0;
				_cmdvel_buff.linear.x = target_linear_vel;
				_cmdvel_buff.angular.z = target_angular_vel;
				//if(control_style == "auto")
				//{
				//	std_msgs::String ctrstyle;
				//	control_style = "manual";
				//	ctrstyle.data = control_style;
				//	control_style_pub.publish(ctrstyle);
				//}
			}

//			if((0 < _feedback_msg.FP1_Dis || 0 < _feedback_msg.FP2_Dis) && (_feedback_msg.FP1_Dis < FP_Obstacle_Limit_1 || _feedback_msg.FP2_Dis < FP_Obstacle_Limit_2) && LowObstacle_enable_ == true )
//			{
//				fp_wd = 0x08;
//				_equipmentInfo.ObstacleWarning = high_wd | low_wd | ultra_wd | fp_wd;
//			}else{
//				fp_wd = 0x00;
//			}


			if((_feedback_msg.FP1_Dis > FP_Limit_ || _feedback_msg.FP2_Dis > FP_Limit_)
					&&(_feedback_msg.FP1_Dis < FP_Limit_Max_ || _feedback_msg.FP2_Dis < FP_Limit_Max_) && FP_soft_enable_ == true){
				_equipmentInfo.FP_Alarm = 1;//防跌落信息
                                fp_wd = 8;
			}else{
				_equipmentInfo.FP_Alarm = 0;
                                fp_wd = 0;
			}

//			if(_equipmentInfo.ObstacleWarning > 0x00 /*|| _equipmentInfo.FP_Alarm != 0x00*/)
//			{
//				//停车命令
//				//电机抱闸
//				if(target_linear_vel > 0){
//					target_linear_vel = 0.0;
//				}
//				target_angular_vel = 0.0;
//				_cmdvel_buff.linear.x = target_linear_vel;
//				_cmdvel_buff.angular.z = target_angular_vel;
//			}
			obstacle_stop_state_pub.publish(stop_state);

			_cmdvel.linear.x = _cmdvel_buff.linear.x*linear_factor_;

			if(_cmdvel_buff.angular.z > 0)
			{
				_cmdvel.angular.z = _cmdvel_buff.angular.z*argular_factor_left;
			}else{
				_cmdvel.angular.z = _cmdvel_buff.angular.z*argular_factor_right;
			}


			//线速度叫速度转双轮差速
			calc_speed();
			//十进制转十六进制
			/////////////////
			////左侧轮速
			/////////////////
			float left_speed_rpm = left_speed * ticks_meter * 60 / wheel_lenght;
			int left_speed_rpm_int = left_speed_rpm;
			control_msg.MOTOR_LWS = left_speed_rpm_int;
			ROS_WARN_DELAYED_THROTTLE(2.0,"left_speed=%.4f, left_speed_rpm_int=%d", left_speed, left_speed_rpm_int);
			//////////////////
			////右侧轮速
			//////////////////
			float right_speed_rpm = right_speed * ticks_meter * 60 / wheel_lenght;
			int right_speed_rpm_int = right_speed_rpm;
			control_msg.MOTOR_RWS = right_speed_rpm_int;
			ROS_WARN_DELAYED_THROTTLE(2.0,"right_speed=%.4f,right_speed_rpm_int=%d", right_speed, right_speed_rpm_int);

			obstacle_pub.publish(obstacle_data);
		}
	}

	void Model_Trx::slowdown(geometry_msgs::Twist& cmd_vel)
	{
		  if(cmd_vel.linear.x >= min_vel_x_chassis_)
		  {
			  cmd_vel.linear.x = std::max( min_vel_x_chassis_, cmd_vel.linear.x - acc_lim_xDC_chassis_ * 2);
			  //cmd_vel.angular.z = 0.0;
		  }
		  else
		  {
			  //cmd_vel.linear.x = std::min(min_vel_x, cmd_vel.linear.x + acc_lim_x_);
			  //cmd_vel.angular.z = 0.0;
		  }
	}

	void Model_Trx::stopcar(geometry_msgs::Twist& cmd_vel){
		  if(cmd_vel.linear.x > 0.1)
		  {
			  cmd_vel.linear.x = std::max( 0.0, cmd_vel.linear.x - acc_lim_xDC_chassis_ * 2);
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


	void Model_Trx::Can_Received()
	{
		ROS_INFO("void Model_Kinco::CanReceived(const can_frame& frame)");

		while (nh_.ok())//
		{
			ros::spinOnce();
			rate.sleep();

			imu_can.Readcan_msg(frame_r[0]);
			imu_can.Readcan_msg(frame_r[1]);
			imu_can.Readcan_msg(frame_r[2]);
			imu_can.Readcan_msg(frame_r[3]);

//			if((frame_r.can_id & CAN_SFF_MASK) == 0x181 || (frame_r.can_id & CAN_SFF_MASK) == 0x281 || (frame_r.can_id & CAN_SFF_MASK) == 0x381 || (frame_r.can_id & CAN_SFF_MASK) == 0x481)
//			{
//				for(int i=0; i<8; i++){
//					ROS_INFO("can_id<0x%03x>read content frame_r.data[%d]= 0x%02x", frame_r.can_id, i, frame_r.data[i]);
//				}
//			}

			/*
			  Chanel 1 --> Gyroscope X          ||  Chanel 2 --> Gyroscope Y          ||  Chanel 3 --> Gyroscope Z
			  Chanel 4 --> Euler angle X        ||  Chanel 5 --> Euler angle Y        ||  Chanel 6 --> Euler angle Z
			  Chanel 7 --> Lin. acceleration X  ||  Chanel 8 --> Lin. acceleration Y  ||  Chanel 9 --> Lin. acceleration Z
			  Chanel 10 --> Magnetometer X      ||  Chanel 11 --> Magnetometer Y      ||  Chanel 12 --> Magnetometer Z
			  Chanel 13 --> Quaternion W        ||  Chanel 14 --> Quaternion X        ||  Chanel 15 --> Quaternion Y      ||  Chanel 16 --> Quaternion Z
			*/

			/*
			    ID |   Chanel
			  181h :  1, 2, 3, 4
			  281h :  5, 6, 7, 8
			  381h :  9, 10, 11, 12
			  481h :  13, 14, 15, 16
			*/

			if((frame_r[0].can_id & CAN_SFF_MASK) == 0x181)
			{
				imuMsg.angular_velocity.x = ((short)((frame_r[0].data[1]<<8) | frame_r[0].data[0]))*0.0001;
				imuMsg.angular_velocity.y = ((short)((frame_r[0].data[3]<<8) | frame_r[0].data[2]))*0.0001;
				imuMsg.angular_velocity.z = ((short)((frame_r[0].data[5]<<8) | frame_r[0].data[4]))*0.0001;
				yaw_data.x = ((short)(frame_r[0].data[7]<<8 | frame_r[0].data[8]))*0.0001;
			}

			if((frame_r[1].can_id & CAN_SFF_MASK) == 0x281)
			{
				yaw_data.y = ((short)(frame_r[1].data[1]<<8 | frame_r[1].data[0]))*0.0001;
				yaw_data.z = ((short)(frame_r[1].data[3]<<8 | frame_r[1].data[2]))*0.0001;  //利用ros定义的imu消息数据结构将yaw角发布给odom
				imuMsg.linear_acceleration.x = ((short)((frame_r[1].data[5]<<8) | frame_r[1].data[4]))*0.0001;
				imuMsg.linear_acceleration.y = ((short)((frame_r[1].data[7]<<8) | frame_r[1].data[6]))*0.0001;
			}

			if((frame_r[2].can_id & CAN_SFF_MASK) == 0x381)
			{
				imuMsg.linear_acceleration.z = ((short)((frame_r[2].data[1]<<8) | frame_r[2].data[0]))*0.0001;
			}

			if((frame_r[2].can_id & CAN_SFF_MASK) == 0x481)
			{
				imuMsg.orientation.w = ((short)(frame_r[3].data[1]<<8 | frame_r[3].data[0]))*0.0001;
				imuMsg.orientation.x = ((short)(frame_r[3].data[3]<<8 | frame_r[3].data[2]))*0.0001;
				imuMsg.orientation.y = ((short)(frame_r[3].data[5]<<8 | frame_r[3].data[4]))*0.0001;
				imuMsg.orientation.z = ((short)(frame_r[3].data[7]<<8 | frame_r[3].data[6]))*0.0001;
			}

			imuMsg.orientation_covariance[0] = 0.0025;
			imuMsg.orientation_covariance[4] = 0.0025;
			imuMsg.orientation_covariance[8] = 0.0025;

			imuMsg.angular_velocity_covariance[0] = 0.0025;
			imuMsg.angular_velocity_covariance[4] = 0.0025;
			imuMsg.angular_velocity_covariance[8] = 0.0025;

			imuMsg.linear_acceleration_covariance[0] = 0.0025;
			imuMsg.linear_acceleration_covariance[4] = 0.0025;
			imuMsg.linear_acceleration_covariance[8] = 0.0025;

			imuMsg.header.frame_id = "imu_link";
			imuMsg.header.stamp = ros::Time::now();

			//ROS_INFO("angular_velocity: %f, %f, %f", imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z);
			//ROS_INFO("imu.linear_acceleration.z: %f, %f, %f", imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z);
			//ROS_INFO("Yaw: %f", yaw_data.z);
		}
	}

	void Model_Trx::Can_Send()
	{
		ROS_INFO("void Model_Kinco::CanReceived(const can_frame& frame)");
		ros::Rate imu_send_rate(20);
		while (nh_.ok())//
		{
			ros::spinOnce();
			imu_send_rate.sleep();
			imu_pub.publish(imuMsg);
			imu_yaw_pub.publish(yaw_data);
		}
	}

	void  Model_Trx::feedback_msg_suber(const carbot_msgs::FeedbackMsg& feedback_msg)
	{
		_feedback_msg = feedback_msg;
		static float lfront, lback , rfront , rback ;
		lfront = feedback_msg.LF_Encoder * wheel_lenght / (ticks_meter * 60);
		lback =  feedback_msg.LB_Encoder * wheel_lenght / (ticks_meter * 60);
		rfront = -feedback_msg.RF_Encoder * wheel_lenght / (ticks_meter * 60);
		rback =  -feedback_msg.RB_Encoder * wheel_lenght / (ticks_meter * 60);
		//ROS_INFO_DELAYED_THROTTLE(2.0,"lfront=%.3f\n,lback=%.3f\n,rfront=%.3f\n,rback=%.3f\n",lfront,lback,rfront,rback);
		float leftrealspeed = (lfront + lback)/(2.0*left_actor);
		float rightrealspeed = (rfront + rback)/(2.0*right_actor);
		//ROS_INFO_DELAYED_THROTTLE(2.0,"leftrealspeed=%.3f\n,rightrealspeed=%.3f\n",leftrealspeed,rightrealspeed);
		_realvel.linear.x = (leftrealspeed + rightrealspeed)/(2*linear_factor_);
		//ROS_INFO_DELAYED_THROTTLE(2.0,"_realvel.linear.x=%.3f\n",_realvel.linear.x);
		if(rightrealspeed > leftrealspeed){
			_realvel.angular.z = (rightrealspeed - leftrealspeed)/(base_wide*argular_factor_left);
		}else{
			_realvel.angular.z = (rightrealspeed - leftrealspeed)/(base_wide*argular_factor_right);
		}
		vel_pub_.publish(_realvel);
	}

	void Model_Trx::feedback_msg_Received()
	{
		ROS_WARN("void Model_Trx::rs485_Received()");
		while (nh_.ok())//
		{
			ros::spinOnce();
			rate_encoder.sleep();
		}
	}

	void Model_Trx::control_msg_send()
	{
		ROS_WARN("void Model_Trx::control_msg_send()");

		while (nh_.ok())//
		{
			ros::spinOnce();
			rate_send.sleep();

			unsigned char hex_bff[1] = "";
			rs485.send_data.assign(0);
			//包头帧
			rs485.send_buf[0] = 0x66;
			rs485.send_buf[1] = 0xaa;
			//帧长度
			rs485.send_buf[2] = 0x0b;
			//传感器使能控制字
			control_msg.EN_CF = 0x01; //使能前碰撞
			control_msg.EN_BF = 0x00; //使能后碰撞
			control_msg.EN_FOBT = 0x00; //使能前超声波避障
			control_msg.EN_ES = 0x01; //使能急停
			control_msg.EN_FP = 0x00; //使能防跌落

			int enable_wd = int(control_msg.EN_FP << 4) + int(control_msg.EN_ES<< 3) +  int(control_msg.EN_FOBT<< 2)
					+ int(control_msg.EN_BF << 1) + int(control_msg.EN_CF);
			DectoHex(enable_wd, &rs485.send_buf[3], 1);
			//电机控制字
			if(_feedback_msg.ES_Alarm == 0x01){
				control_msg.MOTOR_CW = 0x03;
			}else{
				control_msg.MOTOR_CW = 0x01;
			}

			//电机控制字
//			if(_feedback_msg.ES_Alarm == 0x01 )
//			{
//				control_msg.MOTOR_CW = 0x03;
//			}else{
//				control_msg.MOTOR_CW = 0x01;
//				if(_feedback_msg.BC_Status == 1)
//				{
//					if(control_style == "auto" && taskgoal_ == _DockPile)
//					{
//						target_linear_vel = 0.0;
//						target_angular_vel = 0.0;
//						_cmdvel_buff.linear.x = target_linear_vel;
//						_cmdvel_buff.angular.z = target_angular_vel;
//						taskgoal_ = "";
//						std_msgs::String ctrstyle;
//						control_style = "manual";
//						ctrstyle.data = control_style;
//						control_style_pub.publish(ctrstyle);
////						std_msgs::String debuginfo;
////						debuginfo.data = control_style + " BC_Status";
////						debug_pub.publish(debuginfo);
//					}
//					control_msg.MOTOR_CW = 0x03;
//					ROS_ERROR_DELAYED_THROTTLE(1.0,"DOCK 1 MOTOR_CW= %d",control_msg.MOTOR_CW);
//				}
//				if(control_style == "auto" && taskgoal_ != _DockPile)
//				{
//					control_msg.MOTOR_CW = 0x01;
//					ROS_ERROR_DELAYED_THROTTLE(1.0,"DOCK 2 MOTOR_CW= %d",control_msg.MOTOR_CW);
//				}

			DectoHex(control_msg.MOTOR_CW, &rs485.send_buf[4], 1);
			//左边电机速度 2byte
			DectoHex(control_msg.MOTOR_LWS, &rs485.send_buf[5], 2);
			//右边电机速度 2byte
			DectoHex(control_msg.MOTOR_RWS, &rs485.send_buf[7], 2);
			//防跌落距离设定
			control_msg.FP_DIS = FP_Limit_;
			DectoHex(control_msg.FP_DIS, &rs485.send_buf[9], 1);
			//三色灯控制字
			//control_msg.LED_CW = 5;
			DectoHex(control_msg.LED_CW, &rs485.send_buf[10], 1);
			//前避障距离设定
			control_msg.FOBT_DIS = FOBT_DIS_;
			DectoHex(control_msg.FOBT_DIS, &rs485.send_buf[11], 1);
			rs485.send_buf[12] = 0x00;
			rs485.send_buf[13] = crc_check(rs485.send_buf,13);//0x5d;
			rs485.send_buf[14] = '\0';
			std::string res;
			res.resize(14);
			res.assign((char*)rs485.send_buf,14);
			rs485.writedata(res);

//			for(int i = 0; i < 14; i++)
//			{
//				ROS_INFO("send content info_cmd_send._pmsg[%d]= 0x%02x",i,rs485.send_buf[i]);
//			}
		}
	}
}


