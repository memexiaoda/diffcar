#ifndef INTERFACE_CAN_H
#define INTERFACE_CAN_H

#include <libsocketcan.h>

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>


#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include "config.h"


#ifdef __cplusplus
extern "C" {
#endif

namespace Chassis_Driver
{

class _interface_can
{

#define BUF_SIZ	(255)

public:
	_interface_can(){};
	virtual ~_interface_can(){};
//	_interface_can& operator = (const _interface_can& other);
//	_interface_can(const _interface_can& other);
    virtual bool FindCan(int& port) = 0;
	virtual void Setcandown(const int port) = 0;
	virtual void Setcanup(const int port) = 0;
	virtual void Setcanbitrate(const int bitrate,const int port) = 0;
	virtual int  add_filter(int id, int mask) = 0;
	virtual bool Initcan_interface(int port,const int bitrate) = 0;
	virtual void prinf_canframe(can_frame& frame) = 0;
	//cansend
};

class model_canopen : public _interface_can
{
public:
	model_canopen():
		filter(NULL),
		filter_count(0),
		port(0),
		out(stdout),
		family(PF_CAN),
		type(SOCK_RAW),
		proto(CAN_RAW),
		s(-1)
	{
		for(int k = 0; k < BUF_SIZ; k++){
			buf[k] = 0;
		}

		n=0,err=0;
		nbytes=0;
		id=0x000, mask=0;
		interface =new char[16];
		sprintf(interface, "can%d", port);

		loopcount = 1;
		infinite = 0;
		dlc=0, rtr=0, extended=0;
	};

	virtual ~model_canopen()
	{
		if(filter != NULL)
		{
			delete[] filter;
			filter = NULL;
		}
	};

	model_canopen(const model_canopen& other);
	model_canopen& operator = (const model_canopen& other);
	bool Init_model(int can_port,int bitrate,can_filter* filter,int filter_count=0);
    virtual bool FindCan(int& port);
	virtual void Setcandown(const int port);
	virtual void Setcanup(const int port);
	virtual void Setcanbitrate(const int bitrate,const int port);
	virtual int add_filter(int id, int mask);
	virtual bool Initcan_interface(int port,const int bitrate);
	virtual void prinf_canframe(can_frame& frame);
public:
	struct can_filter *filter;
	int filter_count;
	int port;

	struct can_frame frame;
	struct ifreq ifr;
	struct sockaddr_can addr;

	FILE *out;
	char *interface;

	char buf[BUF_SIZ];
	int family , type , proto ;
	int	s;
	//candump
	int n, err;
	int nbytes;
	int id, mask;
public:
    //Candump
	bool Initdump();
	bool Readcan_msg(can_frame& frame);
	//Cansend
	int loopcount, infinite;
	int dlc, rtr, extended;

	bool Initsend();
	void Fillcanframe(can_frame& frame,__u32 can_id, __u8 command,__u8 index_l,__u8 index_h,__u8 sub_index, __u8 can_dlc,__u8 data1=0, __u8 data2=0,__u8 data3=0,__u8 data4=0);
	bool Sendcan_msg(can_frame& frame,int extended = 0,int rtr = 0);
};

}

#ifdef __cplusplus
}
#endif

#endif
