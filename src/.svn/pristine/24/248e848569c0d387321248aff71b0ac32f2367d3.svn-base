#include "model_com/interface_can.h"
#include <signal.h>
//#include <math.h>
//#include <stdlib.h>
//#include <assert.h>
//#include <unistd.h>
using namespace Chassis_Driver;


model_canopen::model_canopen(const model_canopen& other)
{
	filter = other.filter;
	filter_count = other.filter_count;
	port = other.port;
	family = other.family;
	type = other.type;
	proto = other.proto;
	memcpy(buf,other.buf,sizeof(char*)*(BUF_SIZ+1));
	nbytes = other.nbytes;
	//i = other.i;
	id = other.id;
	mask = other.mask;
	interface = other.interface;
	s = other.s;
	n = other.n;
	err = other.err;
	out = other.out;
}

bool model_canopen::Init_model(int can_port,int bitrate,can_filter* filter,int filter_count){

//	char	l_c8Command[64] = {0};
//	sprintf(l_c8Command, "sudo modprobe can");
//	system(l_c8Command);
//	ROS_WARN("sudo modprobe can");
//	memset(l_c8Command, 0, sizeof(l_c8Command));
//	sprintf(l_c8Command, "sudo modprobe can_raw");
//	system(l_c8Command);
//	ROS_WARN("sudo modprobe can_raw");
//	memset(l_c8Command, 0, sizeof(l_c8Command));
//	sprintf(l_c8Command, "sudo modprobe mttcan");
//	system(l_c8Command);
//	ROS_WARN("sudo modprobe mttcan");
//	sprintf(interface, "can%d",can_port);
//	ROS_WARN(interface);

	if(!Initcan_interface(can_port,bitrate)){
		ROS_WARN("Initcan_interface fail");
		return false;
	}
	ROS_WARN("Initcan_interface success");
	for(int i= 0; i < filter_count; i++){

		add_filter(filter[i].can_id,filter[i].can_mask);
	}

	if(!Initdump())
	{
		ROS_WARN("Initdump fail");
		return false;
	}

	ROS_WARN("Initdump success");
//
//	if(!Initsend()){
//		ROS_WARN("Initsend fail");
//		return false;
//	}
//
//	ROS_WARN("Initsend success");
}

int model_canopen::add_filter(int id, int mask)
{
	filter = (can_filter*)realloc(filter, sizeof(struct can_filter) * (filter_count + 1));
	if(!filter)
		return -1;


	filter[filter_count].can_id = id;
	filter[filter_count].can_mask = mask;
	filter_count++;

	for(int i = 0 ;i < filter_count; i++){
		printf("id: 0x%03x mask: 0x%03x count:%d \n", filter[i].can_id, filter[i].can_mask,filter_count);
	}

	return 0;
}

bool model_canopen::Initcan_interface(int port,const int bitrate)
{
	signal(SIGPIPE, SIG_IGN);
	//	port = 0;
	//	while (!FindCan(port))
	//	{
	//		ROS_WARN("FindCan port:%d",port);
	//		port++;
	//		return  false;
	//	}
//	ROS_INFO("Initcan_interface port=%d",port);
	FindCan(port);
//	Setcandown(port);
//	ROS_INFO("bitrate=%d", bitrate);
//	Setcanbitrate(bitrate, port);
//	Setcanup(port);

	return true;
}

bool model_canopen::FindCan(int& port)
{
	//char buf[128] = {0};
//	char	l_c8Command[64] = {0};
//	sprintf(l_c8Command, "sudo modprobe can", port);
//	system(l_c8Command);
//	ROS_WARN("sudo modprobe can");
//	memset(l_c8Command, 0, sizeof(l_c8Command));
//	sprintf(l_c8Command, "sudo modprobe can_raw", port);
//	system(l_c8Command);
//	ROS_WARN("sudo modprobe can_raw");
//	memset(l_c8Command, 0, sizeof(l_c8Command));
//	sprintf(l_c8Command, "sudo modprobe mttcan", port);
//	system(l_c8Command);
//	ROS_WARN("sudo modprobe mttcan");
	sprintf(interface, "can%d", port);

	if(can_do_stop(interface) == 0){
		ROS_WARN(interface);
		return true;
	}

	return true;
}

void model_canopen::Setcanup(const int port)
{
#ifndef WIN32
    char	l_c8Command[64] = {0};
    sprintf(l_c8Command, "sudo ip link set can%d up", port);
    system(l_c8Command);
    ROS_WARN("sudo ip link set can%d up",port);
#endif
}

void model_canopen::Setcandown(const int port)
{
#ifndef WIN32
    char	l_c8Command[64] = {0};
    sprintf(l_c8Command, "sudo ip link set can%d down", port);
    system(l_c8Command);
    ROS_WARN("sudo ip link set can%d down",port);
#endif
}

void model_canopen::Setcanbitrate(const int bitrate,const int port)
{
#define TX_QUEUE_LEN		4096

#ifndef WIN32
	char l_c8Command[128] = {0};
	sprintf(l_c8Command, "sudo ip link set can%d type can bitrate %d", port, bitrate);//"echo %d > /sys/class/net/can%d/can_bittiming/bitrate"
	system(l_c8Command);
	ROS_WARN("sudo ip link set can%d type can bitrate %d", port, bitrate);
//	memset(l_c8Command, 0, sizeof(l_c8Command));
//	sprintf(l_c8Command, "sudo echo %d > /sys/class/net/can%d/tx_queue_len", TX_QUEUE_LEN, port);
//	system(l_c8Command);
#endif
}

void model_canopen::prinf_canframe(can_frame& frame)
{
	if (frame.can_id & CAN_EFF_FLAG)
	{
		n = snprintf(buf, BUF_SIZ, "<0x%08x> ", frame.can_id & CAN_EFF_MASK);
	}
	else
	{
		n = snprintf(buf, BUF_SIZ, "<0x%03x> ", frame.can_id & CAN_SFF_MASK);
		//ROS_WARN("0x 03x s",buf);
	}

	n += snprintf(buf + n, BUF_SIZ - n, "[%d]", frame.can_dlc);

	for (int i = 0; i < frame.can_dlc; i++) {
		n += snprintf(buf + n, BUF_SIZ - n, "%02x ", frame.data[i]);
	}

	if (frame.can_id & CAN_RTR_FLAG)
		n += snprintf(buf + n, BUF_SIZ - n, "remote request");

	//fprintf(out, "%s\n", buf);
	//ROS_WARN("%s",buf);
}

/////////////////
//
/////////////////


bool model_canopen::Initdump()
{
	if ((s = socket(family, type, proto)) < 0) {
		//perror("socket");
		ROS_ERROR("perror(socket)");
		return false;
	}

	addr.can_family = family;

	strncpy(ifr.ifr_name, interface, sizeof(ifr.ifr_name));

	if (ioctl(s, SIOCGIFINDEX, &ifr)) {
		//perror("ioctl");
		ROS_ERROR("ioctl");
		return false;
	}

	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		//perror("bind");
		ROS_ERROR("bind");
		return false;
	}

	if (filter) {
		if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filter,
				   filter_count * sizeof(struct can_filter)) != 0) {
			//perror("setsockopt");
			ROS_ERROR("setsockopt");
			//exit(1);
			return false;
		}
	}

	return true;
}


bool model_canopen::Readcan_msg(can_frame& frame)
{
	int nbytes = -1;
	signal(SIGPIPE, SIG_IGN);
	if ((nbytes = read(s, &frame, sizeof(struct can_frame))) < 0) {
		//perror("read");
		ROS_ERROR("perror(read)");
		//return false;
		return false;
	} else {
		//ROS_ERROR("222222222222222");
		//prinf_canframe(frame);
	}
	return true;
}

/////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////

bool model_canopen::Initsend(){

	if ((s = socket(family, type, proto)) < 0) {
		//perror("socket");
		ROS_ERROR("perror(socket)");
		return false;
	}

	addr.can_family = family;
	strncpy(ifr.ifr_name, interface, sizeof(ifr.ifr_name));

	if (ioctl(s, SIOCGIFINDEX, &ifr)) {
		//perror("ioctl");
		ROS_ERROR("ioctl");
		return false;
	}

	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		//perror("bind");
		ROS_ERROR("bind");
		return false;
	}

	if (filter) {
		if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filter,
				   filter_count * sizeof(struct can_filter)) != 0) {
			//perror("setsockopt");
			ROS_ERROR("setsockopt");
			return false;
			//exit(1);
		}
	}

	return true;
}

//int model_canopen::Cansendthreadfunc(void* pnh_)
//{
//	ROS_WARN("Cansend::Cansendthreadfunc");
//
//	loopcount = 10;
//
//	while ((*(ros::NodeHandle*) pnh_).ok())//(*(ros::NodeHandle*) pnh_).ok()
//	{
//		//boost::this_thread::interruption_point();
//		//ROS_WARN("Cansend::Cansendthreadfunc");
//
//		ros::spinOnce();
//		usleep(1);
//
//		int ret = 0;
//		int dlc = 8;
//
//		if(loopcount < 1){
//			continue;
//		}
//
//		frame.can_id = 0x601;
//		frame.data[0] = 0x09;
//		frame.data[1] = 0x09;
//		frame.data[2] = 0x09;
//		frame.data[3] = 0x08;
//		frame.data[4] = 0x08;
//		frame.data[5] = 0x08;
//		frame.data[6] = 0x07;
//		frame.data[7] = 0x07;
//
//		frame.can_dlc = dlc;
//
//		if (extended) {
//			frame.can_id &= CAN_EFF_MASK;
//			frame.can_id |= CAN_EFF_FLAG;
//		} else {
//			frame.can_id &= CAN_SFF_MASK;
//		}
//
//		if (rtr)
//			frame.can_id |= CAN_RTR_FLAG;
//
//		while (infinite || loopcount--) {
//			if(!Sendcan_msg(frame))
//				break;
//			usleep(1000);
//			ROS_ERROR("send loopcount%d",loopcount);
//		}
//		close(s);
//	}
//	cleanup:
//	ROS_INFO("Cansendthreadfunc shut down.");
//		Sendthreadstate = 0;
//		return Sendthreadstate;
//	return true;
//}



bool model_canopen::Sendcan_msg(can_frame& frame,int extended,int rtr)
{
	if (extended) {
		frame.can_id &= CAN_EFF_MASK;
		frame.can_id |= CAN_EFF_FLAG;
	} else {
		frame.can_id &= CAN_SFF_MASK;
	}

	if (rtr)
		frame.can_id |= CAN_RTR_FLAG;

	int ret=0;
	ret = write(s, &frame, sizeof(frame));
	if (ret == -1) {
		//perror("write");
		ROS_ERROR("Sendcan_msg error");
		return false;
	}
	return true;
}

void model_canopen::Fillcanframe(can_frame& frame, __u32 can_id, __u8 command,__u8 index_l,__u8 index_h,__u8 sub_index,__u8 can_dlc,__u8 data1, __u8 data2,__u8 data3,__u8 data4)
{
	frame.can_id = can_id;
	frame.data[0] = command, frame.data[1] = index_l, frame.data[2] = index_h, frame.data[3] = sub_index;
	frame.data[4] = data1, frame.data[5] = data2, frame.data[6] = data3, frame.data[7] = data4;
	frame.can_dlc = can_dlc;
}


