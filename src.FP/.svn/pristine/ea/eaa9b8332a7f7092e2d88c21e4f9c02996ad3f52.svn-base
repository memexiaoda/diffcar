#include "model_chassis/model_kinco.h"
#include <math.h>
#include <stdlib.h>


using namespace Chassis_Driver;

bool Model_Kinco::Init_model()
{
	if(!Kinco_can.Initcan_interface(0,can_bitrate)){
		ROS_WARN("Initcan_interface fail");
		return false;
	}

	ROS_WARN("Initcan_interface success");
	Kinco_can.add_filter(filter_start_id,filter_start_mask);
	//ROS_WARN("filter_start_id=0x%03");
	Kinco_can.add_filter(filter_stop_id,filter_stop_mask);

	if(!Kinco_can.Initdump())
	{
		ROS_WARN("Initdump fail");
		return false;
	}

	ROS_WARN("Initdump success");

	Kinco_can.add_filter(filter_start_id,filter_start_mask);
	Kinco_can.add_filter(filter_stop_id,filter_stop_mask);

	if(!Kinco_can.Initsend()){
		ROS_WARN("Initsend fail");
		return false;
	}

	ROS_WARN("Initsend success");
	//candump.StartdumpThread((void*)&nh_);
	//cansend.StartsendThread((void*)&nh_model);

#ifndef MOTION_MODE
	Init_leftwheel();
	ROS_WARN("Init_leftwheel success");
	Init_rightwheel();
	ROS_WARN("Init_rightwheel success");
#else
	Init_motionmode();
	ROS_WARN("Init_motionmode success");
#endif
	ROS_WARN("Init_can return");

	rs485.Init_model(dev_serial_rs485.c_str(),port_speed);

	rs485.send_buf[0] = 0xa5;
	rs485.send_buf[1] = 0x03;
	rs485.send_buf[2] = 0x00;
	rs485.send_buf[3] = 0xff;
	rs485.send_buf[4] = 0xfd;
	rs485.send_buf[5] = 0x77;


	int send_len = rs485.Send_msgs(rs485.send_buf,5);
	rate.sleep();

	vel_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/real_vel", 2, true);
	_duration = ros::Duration(1.0/10.0);
	_can_received_filter = boost::bind(&Model_Kinco::Can_Received,this);
	_can_received_thread = new boost::thread(_can_received_filter);
	_can_send_filter = boost::bind(&Model_Kinco::Can_Send,this);
	_can_send_thread = new boost::thread(_can_send_filter);

	_485_received_filter = boost::bind(&Model_Kinco::rs485_Received,this);
	_485_received_thread = new boost::thread(_485_received_filter);
	_485_send_filter = boost::bind(&Model_Kinco::rs485_Send,this);
	_485_send_thread = new boost::thread(_485_send_filter);

	_can_received_thread->join();
	_can_send_thread->join();

	_485_received_thread->join();
	_485_send_thread->join();

	return true;
}


void Model_Kinco::calc_speed()
{
	if(_cmdvel.angular.z > 0.0){
		left_speed = _cmdvel.linear.x - fabs(tan(_cmdvel.angular.z)*base_wide*0.5);
		right_speed = _cmdvel.linear.x + fabs(tan(_cmdvel.angular.z)*base_wide*0.5);
	}
	else if(_cmdvel.angular.z == 0.0){
		left_speed = _cmdvel.linear.x;
		right_speed = _cmdvel.linear.x;
	}
	else
	{
		left_speed = _cmdvel.linear.x + fabs(tan(_cmdvel.angular.z)*base_wide*0.5);
		right_speed = _cmdvel.linear.x - fabs(tan(_cmdvel.angular.z)*base_wide*0.5);
	}
	ROS_WARN("left_speed=%.3f,right_speed=%.3f",left_speed,right_speed);
}

void Model_Kinco::calc_motionspeed(){

}

bool Model_Kinco::Init_leftwheel()
{
	//设定工作模式(3 速度模式）
	frame_s.can_id = 0x602;
	frame_s.data[0] = 0x2f;
	frame_s.data[1] = 0x60;
	frame_s.data[2] = 0x60;
	frame_s.data[3] = 0x00;
	frame_s.data[4] = 0x03;
	frame_s.data[5] = 0x00;
	frame_s.data[6] = 0x00;
	frame_s.data[7] = 0x00;
	frame_s.can_dlc = 5;
	Kinco_can.Sendcan_msg(frame_s);
	//控制字
	frame_s.can_id = 0x602;
	frame_s.data[0] = 0x2f;
	frame_s.data[1] = 0x40;
	frame_s.data[2] = 0x60;
	frame_s.data[3] = 0x00;
	frame_s.data[4] = 0x0f;
	frame_s.data[5] = 0x00;
	frame_s.data[6] = 0x00;
	frame_s.data[7] = 0x00;
	frame_s.can_dlc = 5;
	Kinco_can.Sendcan_msg(frame_s);
	return true;
}


bool Model_Kinco::Init_rightwheel()
{
	//设定工作模式(3 速度模式）
	frame_s.can_id = 0x601;
	frame_s.data[0] = 0x2f;
	frame_s.data[1] = 0x60;
	frame_s.data[2] = 0x60;
	frame_s.data[3] = 0x00;
	frame_s.data[4] = 0x03;
	frame_s.data[5] = 0x00;
	frame_s.data[6] = 0x00;
	frame_s.data[7] = 0x00;
	frame_s.can_dlc = 5;
	Kinco_can.Sendcan_msg(frame_s);

	//控制字
	frame_s.can_id = 0x601;
	frame_s.data[0] = 0x2f;
	frame_s.data[1] = 0x40;
	frame_s.data[2] = 0x60;
	frame_s.data[3] = 0x00;
	frame_s.data[4] = 0x0f;
	frame_s.data[5] = 0x00;
	frame_s.data[6] = 0x00;
	frame_s.data[7] = 0x00;
	frame_s.can_dlc = 5;
	Kinco_can.Sendcan_msg(frame_s);

	return true;
}

bool Model_Kinco::Init_motionmode(){
	//设定协同运动使能
	Kinco_can.Fillcanframe(frame_s,0x601,0x2f,0x00,0x43,0x00,5,0x01);
	Kinco_can.Sendcan_msg(frame_s);
	//设定工作模式(33=0x21 协同速度模式）
	Kinco_can.Fillcanframe(frame_s,0x601,0x2f,0x00,0x43,0x01,5,0x21);
	Kinco_can.Sendcan_msg(frame_s);
	//设定协同运动控制字
	Kinco_can.Fillcanframe(frame_s,0x601,0x2f,0x00,0x43,0x02,5,0x0f);
	Kinco_can.Sendcan_msg(frame_s);
	//设定协同运动控制加速度
	Kinco_can.Fillcanframe(frame_s,0x601,0x2f,0x02,0x43,0x01,5,0xc9);
	Kinco_can.Sendcan_msg(frame_s);
	//设定协同运动控制减速度
	Kinco_can.Fillcanframe(frame_s,0x601,0x2f,0x02,0x43,0x02,5,0xc9);
	Kinco_can.Sendcan_msg(frame_s);
	//设定协同运动控制转角加速度
	Kinco_can.Fillcanframe(frame_s,0x601,0x2f,0x03,0x43,0x04,5,0x64);
	Kinco_can.Sendcan_msg(frame_s);
	//设定协同运动控制转角减速度
	Kinco_can.Fillcanframe(frame_s,0x601,0x2f,0x03,0x43,0x04,5,0x64);
	Kinco_can.Sendcan_msg(frame_s);
	//设定协同运动控制最大速度
	Kinco_can.Fillcanframe(frame_s,0x601,0x23,0x02,0x43,0x05,8,0x5b,0xf1,0x01,0x00);
	Kinco_can.Sendcan_msg(frame_s);
	//设定协同运动控制最大转角速度
	Kinco_can.Fillcanframe(frame_s,0x601,0x23,0x03,0x43,0x06,8,0xad,0xf8,0x00,0x00);
	Kinco_can.Sendcan_msg(frame_s);
	return true;
}


void Model_Kinco::velCallback(const geometry_msgs::Twist & cmdvel)
{
	//ROS_INFO("Model_Kinco::velCallback");
	_cmdvel = cmdvel;
	_cmdvel.angular.z = 0.8 * cmdvel.angular.z;
	//ROS_INFO("speed=%.3f,turn=%.3f",_cmdvel.linear.x,_cmdvel.angular.z);
	ROS_INFO("_cmdvel.angular.z=%.3f,cmdvel.angular.z=%.3f",_cmdvel.angular.z,cmdvel.angular.z);
#ifndef MOTION_MODE
	calc_speed();
	ROS_INFO("left_speed=%.3f,right_speed=%.3f",left_speed,right_speed);
	//十进制转十六进制
	unsigned char hex_bff[4] = "";
	int left_speed_rpm = left_speed*60000/(0.15*Pi);
	ROS_INFO("left_speed_rpm=%d,",left_speed_rpm);
	left_speed_rpm = -left_speed_rpm;
	DectoHex(left_speed_rpm, hex_bff, 4);
//	for (int i = 0; i < 4; ++i)
//	{
//		printf("hex_bff[%d] = 0x%02X\n", i, hex_bff[i]);
//	}
	frame_s.can_id = 0x602;
	frame_s.data[0] = 0x23;
	frame_s.data[1] = 0xff;
	frame_s.data[2] = 0x60;
	frame_s.data[3] = 0x00;
	frame_s.data[4] = hex_bff[3];
	//ROS_INFO("left.frame.data[4]=0x%08x,buf[2]= %02x\n,",cansend.frame.data[4],hex_bff[3]);
	frame_s.data[5] = hex_bff[2];//atoi((char*)buf[1]);
	//ROS_INFO("left.frame.data[5]=0x%08x,buf[1]= %02x\n,",cansend.frame.data[5],hex_bff[2]);
	frame_s.data[6] = hex_bff[1];//atoi((char*)buf[0]);
	//ROS_INFO("left.frame.data[6]=0x%08x,buf[0]= %02x\n,",cansend.frame.data[6],hex_bff[1]);
	frame_s.data[7] = hex_bff[0];
	frame_s.can_dlc = 8;
	Kinco_can.prinf_canframe(frame_s);
	Kinco_can.Sendcan_msg(frame_s);

	///////////////////////////////////////
	///
	///////////////////////////////////////
	int right_speed_rpm = right_speed*60000/(0.15*Pi);
	ROS_INFO("right_speed_rpm=%d,",right_speed_rpm);
	DectoHex((unsigned int)right_speed_rpm, hex_bff, 4);

//	for (int i = 0; i < 4; ++i)
//	{
//		printf("right_speed_rpm_convert[%d] = 0x%02X\n", i, hex_bff[i]);
//	}

	frame_s.can_id = 0x601;
	frame_s.data[0] = 0x23;
	frame_s.data[1] = 0xff;
	frame_s.data[2] = 0x60;
	frame_s.data[3] = 0x00;
	frame_s.data[4] = hex_bff[3];
	//ROS_INFO("cansend.frame.data[4]=0x%08x,hex_bff[3]= %02x\n,",cansend.frame.data[4],hex_bff[3]);
	frame_s.data[5] = hex_bff[2];//atoi((char*)buf[1]);
	//ROS_INFO("cansend.frame.data[5]=0x%08x,hex_bff[2]= %02x\n,",cansend.frame.data[5],hex_bff[2]);
	frame_s.data[6] = hex_bff[1];//atoi((char*)buf[0]);
	//ROS_INFO("cansend.frame.data[6]=0x%08x,hex_bff[1]= %02x\n,",cansend.frame.data[6],hex_bff[1]);
	frame_s.data[7] = hex_bff[0];
	//ROS_INFO("cansend.frame.data[7]=0x%08x,hex_bff[0]= %02x\n,",cansend.frame.data[7],hex_bff[0]);
	frame_s.can_dlc = 8;
	Kinco_can.prinf_canframe(frame_s);
	Kinco_can.Sendcan_msg(frame_s);
#else
	//十进制转十六进制
	unsigned char hex_bff[4] = "";
	int motion_speed_rpm = _cmdvel.linear.x * 60000/(0.15*Pi);
	motion_speed_rpm = -motion_speed_rpm;
	DectoHex(motion_speed_rpm, hex_bff, 4);

	for (int i = 0; i < 4; ++i)
	{
		printf("hex_bff[%d] = 0x%02X\n", i, hex_bff[i]);
	}

	cansend.Fillcanframe(frame_s,0x601,0x23,0x02,0x43,0x03,0x08,hex_bff[3],hex_bff[2],hex_bff[1],hex_bff[0]);
	cansend.Sendcan_msg(frame_s);

	int motion_angular = _cmdvel.angular.z * 180 / Pi;

#endif
}

void Model_Kinco::Can_Received()
{
	ROS_INFO("void Model_Kinco::CanReceived(const can_frame& frame)");
	while (nh_.ok())//
	{
		ros::spinOnce();
		//ROS_WARN("spinOnce");
		//usleep(1000*50);
		rate.sleep();
		//_duration.fromSec(0.1);
		//cansend.Fillcanframe(frame,0x601,0x40,0x6c,0x60,0x00,0x08);
		Kinco_can.Readcan_msg(frame_r);
		if((frame_r.can_id & CAN_SFF_MASK) == 0x181 || (frame_r.can_id & CAN_SFF_MASK) == 0x281 || (frame_r.can_id & CAN_SFF_MASK) == 0x381 || (frame_r.can_id & CAN_SFF_MASK) == 0x481)
		{
			for(int i=0; i<8;i++){
				ROS_INFO("can_id<0x%03x>read content frame_r.data[%d]= 0x%02x",frame_r.can_id,i,frame_r.data[i]);
			}
		}
//		if((frame_r.can_id & CAN_SFF_MASK) == 0x582)
//		{
//			if(frame_r.data[0]==0x43 && frame_r.data[1]==0x6c && frame_r.data[2] == 0x60 && frame_r.data[3]==0x00){
//				int left_speed_rpm = HextoDec(&frame_r.data[4],4);
//				ROS_INFO("left_speed_rpm = %d",left_speed_rpm);
//				left_real_speed = left_speed_rpm * (0.15*Pi) / 60000;
//
//				if(fabs(left_real_speed) < 0.01)
//				{
//					left_real_speed = 0.0;
//				}
//				ROS_INFO("left_real_speed = %f",left_real_speed);
//			}
//		}
//
//		if((frame_r.can_id & CAN_SFF_MASK) == 0x581){
//			if(frame_r.data[0]==0x43 && frame_r.data[1]==0x6c && frame_r.data[2] == 0x60 && frame_r.data[3]==0x00){
//				int right_speed_rpm = HextoDec(&frame_r.data[4],4);
//				ROS_INFO("right_speed_rpm = %d",right_speed_rpm);
//				right_real_speed = right_speed_rpm * (0.15*Pi) / 60000;
//
//				if(fabs(right_real_speed) < 0.01)
//				{
//					right_real_speed = 0.0;
//				}
//
//				ROS_INFO("right_real_speed = %f",right_real_speed);
//			}
//		}
//
//		geometry_msgs::Twist vel_real;
//
//		vel_real.linear.x = (left_real_speed + right_real_speed)/2;
//		vel_real.angular.z = (right_real_speed - left_real_speed)/base_wide;
//		//vel_pub_.publish(vel_real);
//		if(fabs(vel_real.linear.x) < 0.01){
//			vel_real.linear.x = 0.0;
//		}
//		if(fabs(vel_real.angular.z) < 0.035){
//			vel_real.angular.z = 0.0;
//		}
//		ROS_INFO("real linear = %f -- real angular = %f",vel_real.linear.x, vel_real.angular.z);


	}


//		cleanup:
			//close(s);
			//ROS_INFO("close(s)");
//			ROS_INFO("Candumpthreadfunc shut down.");
//			Dumpthreadstate = 0;

}

void Model_Kinco::Can_Send()
{
	ROS_INFO("void Model_Kinco::CanReceived(const can_frame& frame)");
	while (nh_.ok())//
	{
		ros::spinOnce();
		//ROS_WARN("spinOnce");
		//usleep(1000);
		rate.sleep();
		//rate_send.sleep();
		//cansend.Fillcanframe(frame_s,0x602,0x40,0x6c,0x60,0x00,0x08);
		//cansend.Sendcan_msg(frame_s);
		//cansend.Fillcanframe(frame_s,0x601,0x40,0x6c,0x60,0x00,0x08);
		//cansend.Sendcan_msg(frame_s);

		////////////////////////////////////////////
		///读取实际编码器速度
		////////////////////////////////////////////
//		cansend.Fillcanframe(frame_s,0x602,0x40,0x6c,0x60,0x00,4);
//		cansend.Sendcan_msg(frame_s);
//
//		cansend.Fillcanframe(frame_s,0x601,0x40,0x6c,0x60,0x00,4);
//		cansend.Sendcan_msg(frame_s);
	}

	Kinco_can.Fillcanframe(frame_s,0x602,0x2f,0xff,0x60,0x00,5);
	Kinco_can.Sendcan_msg(frame_s);
	Kinco_can.Fillcanframe(frame_s,0x601,0x2f,0xff,0x60,0x00,5);
	Kinco_can.Sendcan_msg(frame_s);
	Kinco_can.Fillcanframe(frame_s,0x602,0x2f,0x40,0x60,0x00,5,0x06);
	Kinco_can.Sendcan_msg(frame_s);
	Kinco_can.Fillcanframe(frame_s,0x601,0x2f,0x40,0x60,0x00,5,0x06);
	Kinco_can.Sendcan_msg(frame_s);
}


void Model_Kinco::rs485_Received()
{
	while (nh_.ok())//
	{
		ros::spinOnce();
		//ROS_WARN("spinOnce");
		//usleep(1000);
		rate.sleep();
//		int rev_len;
//		if((rev_len = rs485.Recv_msgs(rs485.rev_buf,512))==-1){
//			ROS_ERROR("rev_len=%d",rev_len);
//			ROS_ERROR("r485 read error!\n");
//		}else{
//			for(int i=0; i< rev_len;i++)
//				ROS_INFO("read content rev_buf[%d]= 0x%02x",i,rs485.rev_buf[i]);
//		}
	}
}


void Model_Kinco::rs485_Send()
{

}

