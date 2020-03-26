#ifndef MODEL_KINCO_H
#define MODEL_KINCO_H

#include "model_com/interface_can.h"
#include "model_com/interface_485.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#ifdef __cplusplus
extern "C" {
#endif

namespace Chassis_Driver
{

/////////////////////////////////////////////////////////
//
//功能：十进制转十六进制
//
//输入：int dec                     待转换的十进制数据
//      int length                  转换后的十六进制数据长度
//
//输出：unsigned char *hex          转换后的十六进制数据
//
//返回：0    success
//
//思路：原理同十六进制转十进制
//////////////////////////////////////////////////////////
int DectoHex(unsigned int dec, unsigned char *hex, int length)
{
    int i;
    for (i = length - 1; i >= 0; i--)
    {
        hex[i] = (dec % 256) & 0xFF;
        dec /= 256;
    }
    return 0;
}

/////////////////////////////////////////////////////
//
//功能：二进制取反
//
//输入：const unsigned char *src  二进制数据
//      int length                待转换的二进制数据长度
//
//输出：unsigned char *dst        取反后的二进制数据
//
//返回：0    success
//
//////////////////////////////////////////////////////
int convert(unsigned char *dst, const unsigned char *src, int length)
{
    int i;
    for (i = 0; i < length; i++)
    {
        dst[i] = src[i] ^ 0xFF;
    }
    return 0;
}

//////////////////////////////////////////////////////////
//
//功能：十六进制转为十进制
//
//输入：const unsigned char *hex         待转换的十六进制数据
//      int length                       十六进制数据长度
//
//输出：
//
//返回：int  rslt                        转换后的十进制数据
//
//思路：十六进制每个字符位所表示的十进制数的范围是0 ~255，进制为256
//      左移8位(<<8)等价乘以256
//
//高位后低位前 倒序
/////////////////////////////////////////////////////////
int HextoDecDisc(unsigned char *hex, int length)
{
    int i;
    short rslt = 0;
    for (i = 0; i < length; i++)
    {
        rslt += (short)(hex[i]) << (8 * i);
    }
    return rslt;
}

//////////////////////////////////////////////////////////
//高位前低位后 正序
//////////////////////////////////////////////////////////
int HextoDec(unsigned char *hex, int length)
{
    int i;
	int rslt = 0;
	for (i = 0; i < length; i++)
	{
		rslt += (short)(hex[i]) << (8 * (length - 1 - i));
	}
	return rslt;
}

typedef boost::function0<void> CallbackType;//struct can_frame

class  Model_Kinco
{

public:
	Model_Kinco():
		base_wide(0.394),
		left_speed(0),
		right_speed(0),
		wheel_lenght(0.15),
		filter_start_id(0x601),
		filter_stop_id(0x602),
		filter_start_mask(1),
		filter_stop_mask(1),
		rate((double)10.0),
		rate_send((double)5.0),
		dev_serial_rs485("/dev/ttyUSB0"),
		port_speed(115200)
	{};

	virtual ~Model_Kinco(){
		_can_received_thread->detach();
		delete _can_received_thread;
		_can_received_thread = NULL;

		_can_send_thread->detach();
		delete _can_send_thread;
		_can_received_thread = NULL;

		_485_received_thread->detach();
		delete _485_received_thread;
		_485_received_thread = NULL;

		_485_send_thread->detach();
		delete _485_send_thread;
		_485_send_thread = NULL;
	}

	bool Init_model();
	ros::NodeHandle nh_;
	geometry_msgs::Twist _cmdvel;
	void velCallback(const geometry_msgs::Twist& cmdvel);

//can接口
public:
//	Candump candump;
//	Cansend cansend;
	model_canopen Kinco_can;
	int filter_start_id;
	int filter_stop_id;
	int filter_start_mask;
	int filter_stop_mask;
	int can_bitrate;

//485接口
public:
	model_rs485 rs485;
	std::string dev_serial_rs485;
	int port_speed;
//运动控制
protected:
	bool Init_leftwheel();
	bool Init_rightwheel();
	bool Init_motionmode();

	void calc_speed();
	void calc_motionspeed();
	void Can_Received();
	void Can_Send();

	void rs485_Received();
	void rs485_Send();
private:
	float left_speed;
	float left_real_speed;
	float right_speed;
	float right_real_speed;
	float base_wide;
	float wheel_lenght;

	CallbackType _can_received_filter;
	CallbackType _can_send_filter;
	boost::thread * _can_received_thread;
	boost::thread * _can_send_thread;


	CallbackType _485_received_filter;
	CallbackType _485_send_filter;
	boost::thread * _485_received_thread;
	boost::thread * _485_send_thread;

	can_frame frame_r;
	can_frame frame_s;

	ros::Rate rate;
	ros::Rate rate_send;
	ros::Duration _duration;
	ros::Publisher vel_pub_;
};

}

#ifdef __cplusplus
}
#endif

#endif
