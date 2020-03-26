#ifndef INTERFACE_485_H
#define INTERFACE_485_H

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>

#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <termios.h>
#include <asm-generic/int-ll64.h>
#include <string>

#include <carbot_msgs/FeedbackMsg.h>

#include "config.h"
#include "AsyncSerial.h"

#ifdef __cplusplus
extern "C" {
#endif


namespace Chassis_Driver
{

#define DEBUGECHO 0

class MAsyncSerialImpl
{
public:
    CallbackAsyncSerial serial;
    std::string receivedData;
};

class OnlineLogMessage
{
public:
	int online;
	int timeout;

	boost::posix_time::ptime time_recvnow;
	boost::posix_time::millisec_posix_time_system_config::time_duration_type time_elapse;

	OnlineLogMessage()
	{
		online = 0;
		timeout=2;
		time_recvnow = boost::posix_time::microsec_clock::universal_time();
	}

	int check_connect()
	{
	    if(online == 1){
	    	//
	    	boost::posix_time::ptime now;
			// ����Ϊ΢��Ϊ��λ;������Խ�microsec_clock�滻��second_clock����Ϊ��λ;
			now = boost::posix_time::microsec_clock::universal_time();
			time_elapse = now - time_recvnow;
			// ����GetTickCount��ֻ����ߵõ�����2��ʱ���ticketֵ�Ĳ��΢��Ϊ��λ;
			int ticks = time_elapse.ticks();
			// �õ�����ʱ����������;
			int sec = time_elapse.total_seconds();
			if(sec >= timeout){
					online = 0;
			}
		}
        return online;
	}

	int Set_connect()
	{
		online=1;
		// ����Ϊ΢��Ϊ��λ;������Խ�microsec_clock�滻��second_clock����Ϊ��λ;
		time_recvnow = boost::posix_time::microsec_clock::universal_time();
	}

	int Set_timeout(int time)
	{
		timeout =  time;
	}
};


// set serial speed
int speed_arr[] = { B115200,B38400, B19200, B9600, B4800, B2400, B1200, B300,\
		B115200,B38400, B19200, B9600, B4800, B2400, B1200, B300,};

int name_arr[] = { 115200,38400, 19200, 9600, 4800, 2400, 1200, 300, 115200,38400, \
                   19200, 9600, 4800, 2400, 1200, 300, };

class _interface_485
{
public:
	_interface_485(){};
	virtual ~_interface_485(){
	};

	virtual int open_dev(char *Dev) = 0;
	virtual void set_speed(int fd, int speed) = 0;
	virtual int set_parity(int fd, int databits, int stopbits, int parity) = 0;
	virtual void config_s(int fd) = 0;
};

class model_rs485 : public _interface_485
{
public:
	model_rs485():
		fp(NULL),
		fd(0),
		com_speed(115200),
		pimpl(new MAsyncSerialImpl),
		private_nh("~")
	{
		memset(rev_buf,0,sizeof(rev_buf));
		memset(send_buf,0,sizeof(send_buf));
		strcpy(dev,"/dev/ttyUSB0");
		feedback_msg_pub = private_nh.advertise<carbot_msgs::FeedbackMsg>("/FeedbackMsg", 1);
	};

	virtual ~model_rs485(){
		if(fp){
			delete fp;
			fp = NULL;
		}
	};

	model_rs485& operator = (const model_rs485& other);
	model_rs485(const model_rs485& other) ;

public:
	bool Init_model(const char* dev_,int port_speed);
	int Send_msgs(const void *__buf, size_t __n);
	int Recv_msgs(void *__buf, size_t __nbytes);

	__u8 rev_buf[512];
	__u8 send_buf[512];
protected:
	virtual int open_dev(char *Dev);
	virtual void set_speed(int fd, int speed);
	virtual int set_parity(int fd, int databits, int stopbits, int parity);
	virtual void config_s(int fd);

private:
	FILE *fp;
	int fd;
	char dev[64];
	int com_speed;
	ros::NodeHandle private_nh;

public:
	  enum {
		  nomal_length = 14 ,
		  nomal_recvlength = 59
	  };

	  enum {
	  	HEAD_H = 0X66,
		HEAD_L = 0XAA,

		TAIL_H = 0X77,
		TAIL_L = 0XCC,

		RECV_HEAD1 = 0X88,
		RECV_HEAD2 = 0XBB,
	  };

	 enum {
	  	SELED_CLOSE = 0X0,
		SELED_RED = 0X1,
		SELED_GREEN = 0X2,
		SELED_BLUE = 0X3,
		SELED_FRED = 0X4,
		SELED_FGREEN = 0X5,
		SELED_FBLUE = 0X6,
	 };
public:
	boost::shared_ptr<MAsyncSerialImpl> pimpl; ///< Pimpl idiom
	OnlineLogMessage uart_onlineMsg;

	//recv
	int recv_ptr;
	int recv_count;
	bool comRecvFlag;
	boost::array<unsigned char,nomal_recvlength+1> Readarray;

	void open(std::string devname, unsigned int baudrate);
	void close();
	bool isOpen();
	bool errorStatus();
	void writedata(std::string data);
	void readCallback(const char *data, size_t size);

	carbot_msgs::FeedbackMsg feedback_msg;
	bool procss_recv_buffer();
	int  Get_check();
	void Get_Sdata();
	void param_init();
	char hex2char(int8_t hex);
	ros::Publisher feedback_msg_pub;
	boost::array<char,nomal_length> send_data;
};


}
#ifdef __cplusplus
}
#endif

#endif
