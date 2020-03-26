#include "interface_485.h"


using namespace Chassis_Driver;

model_rs485::model_rs485(const model_rs485& other){

	fp = other.fp;
	fd = other.fd;
	strcpy(dev,other.dev);//other.dev;
}


bool model_rs485::Init_model(const char* dev_,int port_speed)
{
//#ifndef WIN32
//    char l_c8Command[64] = {0};
//    sprintf(l_c8Command, "sudo chmod 777 %c -R", dev);
//    system(l_c8Command);
//#endif


//	ROS_INFO("dev:%s",dev);
//	fd = open_dev(dev);
//	set_speed(fd,com_speed);
//	ROS_INFO("port_speed:%d",com_speed);
//	config_s(fd);
//	if (set_parity(fd,8,1,'N') == false){
//		printf("set parity error\n");
//		return false;
//	}
//    else{
//        printf("set serial ok!\n");
//    }
	open(dev_,port_speed);
	com_speed = port_speed;
	strcpy(dev,dev_);
	ROS_INFO("dev:%s",dev);
	ROS_INFO("port_speed:%d",com_speed);
	pimpl->receivedData.clear();
	uart_onlineMsg.online=0;
	uart_onlineMsg.Set_timeout(2);

	param_init();
	return true;
}


int model_rs485::open_dev(char *Dev){
//	int fd = open(Dev, O_RDWR);
//
//	if (-1 == fd)
//	{
//		perror("Can't Open Serial Port");
//		return -1;
//	}
//
//	if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
//	{
//		printf("Unable set to NONBLOCK mode");
//		return -1;
//	}

	return fd;
}

void model_rs485::set_speed(int fd, int speed)
{
    int i;
    int status;
    struct termios Opt;

    tcgetattr(fd, &Opt);
    for (i=0; i<sizeof(speed_arr)/sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if (status != 0)
            {
                perror("tcsetattr fd1");
                return;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
}

int model_rs485::set_parity(int fd, int databits, int stopbits, int parity){
	struct termios options;
	if (tcgetattr(fd, &options) != 0)
	{
		perror("SetupSerial 1");
		return(false);
	}

	options.c_cflag &= ~CSIZE;

	//  data bit
	switch (databits)
	{
		case 7:
			options.c_cflag |= CS7;
		break;
		case 8:
			options.c_cflag |= CS8;
		break;
		default:
			fprintf(stderr,"Unsupported data size\n");
		return (false);
	}

	switch (parity)
	{
		case 'n':
		case 'N':
			options.c_cflag &= ~PARENB;   // Clear parity enable
			options.c_iflag &= ~INPCK;    // Enable parity checking
			//
			//options.c_iflag |= INLCR;
			options.c_lflag &= 0;  // add comment it's very important to set this parameter.
			options.c_oflag  &= ~OPOST;   /*Output*/
			break;

		case 'o':
		case 'O':
			options.c_cflag |= (PARODD | PARENB); // set odd parity
			options.c_iflag |= INPCK;             // disnable parity checking
			break;

		case 'e':
		case 'E':
			options.c_cflag |= PARENB;     // enable parity
			options.c_cflag &= ~PARODD;    // change into even
			options.c_iflag |= INPCK;      // disnable parity checking
			break;

		case 'S':
		case 's':  // as no parity
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;
			break;

		default:
			fprintf(stderr,"Unsupported parity\n");
		return (false);
	}

	// set stop bit
	switch (stopbits)
	{
		case 1:
			options.c_cflag &= ~CSTOPB;
		break;
		case 2:
			options.c_cflag |= CSTOPB;
		break;
		default:
			fprintf(stderr,"Unsupported stop bits\n");
		return (false);
	}

	// Set input parity option
	if (parity != 'n')
	{
		options.c_iflag |= INPCK;
		tcflush(fd,TCIFLUSH);
		options.c_cc[VTIME] = 150; // set time out for 15 seconds
		options.c_cc[VMIN] = 0;       // update the options and do it now
	}

	if (tcsetattr(fd,TCSANOW,&options) != 0)
	{
		perror("SetupSerial 3");
		return (false);
	}
	return (true);
}

void model_rs485::config_s(int fd)
{
	struct termios oldtio, newtio;

	tcgetattr(fd, &oldtio);
	memset(&newtio, 0, sizeof(newtio));

	int speed_index;

	if(com_speed == 115200){
		speed_index = B115200;
	}else if(com_speed == 9600){
		speed_index = B9600;
	}

	newtio.c_cflag = speed_index | CS8 | CLOCAL | CREAD | CSTOPB;
	newtio.c_cflag &= ~CRTSCTS;
	newtio.c_iflag = IGNPAR | ICRNL;
	//
	//newtio.c_iflag |= INLCR;
	//
	newtio.c_iflag &= ~( IXON | IXOFF | IXANY );
	newtio.c_oflag = 0;
	//newtio.c_lflag = 0;
	newtio.c_lflag = ICANON;

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);
}

int model_rs485::Send_msgs(const void *__buf, size_t __n){

	return write(fd, __buf, __n);
}

int model_rs485::Recv_msgs(void *__buf, size_t __nbytes){
	return read(fd, __buf, __nbytes);
}





void model_rs485::open(std::string devname, unsigned int baudrate)
{
    try {
        pimpl->serial.open(devname,baudrate);
    } catch(boost::system::system_error&)
    {
        //Errors during open
    }
    pimpl->serial.setCallback(bind(&model_rs485::readCallback,this, _1, _2));
}

void model_rs485::close()
{
    pimpl->serial.clearCallback();
    try
    {
        pimpl->serial.close();
    }
    catch(boost::system::system_error&)
    {
        //Errors during port close
    }
    pimpl->receivedData.clear();//Clear eventual data remaining in read buffer
}

bool model_rs485::isOpen()
{
    return pimpl->serial.isOpen();
}

bool model_rs485::errorStatus()
{
    return pimpl->serial.errorStatus();
}

void model_rs485::writedata(std::string data)
{
    pimpl->serial.writeString(data);
}

void model_rs485::param_init()
{
	memset(&feedback_msg, 0, sizeof(feedback_msg));
}

void model_rs485::readCallback(const char *data, size_t size)
{
    //pimpl->receivedData+=QString::fromAscii(data,size);
        std::string res;
        res.clear();
        res.assign(data,size);
        pimpl->receivedData+=res;
	#if 0
	print_hex(pimpl->receivedData);
	#endif
	procss_recv_buffer();

}

bool model_rs485::procss_recv_buffer()
{

	std::string recvstr = pimpl->receivedData;
	pimpl->receivedData.clear();
	const char* ch=recvstr.c_str();
	int size = recvstr.size();


	for(int index=0; index<size; index++){
		unsigned char data = (unsigned char)ch[index];
	 	switch(recv_ptr){
			case 0:
				if(data==RECV_HEAD1){
						recv_ptr=1;
						recv_count=0;
						Readarray.assign(0);
					}
				break;
			case 1:
				if(data==RECV_HEAD2){
						recv_ptr=2;
						recv_count=0;
						Readarray.assign(0);
					}
				else{
						recv_ptr=0;
						recv_count=0;
						Readarray.assign(0);
					}
				break;
			case 2:
				if(data==(nomal_recvlength-3)){
						recv_ptr=3;
						recv_count=0;
						Readarray.assign(0);
					}
				else{
						recv_ptr=0;
						recv_count=0;
						Readarray.assign(0);
					}
				break;
			case 3:
				Readarray[recv_count]=data;
				recv_count++;
				if(recv_count>=(nomal_recvlength-3)){
						recv_ptr=0;
						recv_count=0;

						if(1==Get_check()){  //���յ�����������
							uart_onlineMsg.Set_connect();

							Get_Sdata();

							Readarray.assign(0);
							return true;
							}
						else{
								std::cout<< dev <<" recv worong !"<<std::endl;
							}
						Readarray.assign(0);
						return false;
					}
				break;
			default:
				recv_ptr=0;
				recv_count=0;
				Readarray.assign(0);
				pimpl->receivedData.clear();
				break;

	 		}
		}
	return false;
}

int  model_rs485::Get_check()
{
	unsigned char * ptr=(unsigned char *)Readarray.c_array();

	unsigned char Data1 = ptr[nomal_recvlength-4];

	unsigned char crc = 0;

	crc +=(unsigned char )RECV_HEAD1;
	crc +=( unsigned char )RECV_HEAD2;
	crc +=( unsigned char )(nomal_recvlength-3);

	for(int i =0;i<(nomal_recvlength-4);i++){

			crc +=ptr[i];
			//std::cout<< "Get_check:ptr = "<<boost::format("%02X ")%(int)ptr[i]<<std::endl;
	}
	#if DEBUGECHO
	std::cout<< "Get_check:Data1 = "<<boost::format("%02X ")%(int)Data1<<std::endl;
	std::cout<< "Get_check:crc = "<<boost::format("%02X ")%(int)crc<<std::endl;
	#endif

	if(Data1 == crc){
		return 1;
	}

	return 0;
}

char model_rs485::hex2char(int8_t hex)
{
	switch(hex){
	case 0:
		return '0';
	case 1:
		return '1';
	case 2:
		return '2';
	case 3:
		return '3';
	case 4:
		return '4';
	case 5:
		return '5';
	case 6:
		return '6';
	case 7:
		return '7';
	case 8:
		return '8';
	case 9:
		return '9';
	case 10:
		return 'A';
	case 11:
		return 'B';
	case 12:
		return 'C';
	case 13:
		return 'D';
	case 14:
		return 'E';
	case 15:
		return 'F';
	}
}

void  model_rs485::Get_Sdata()
{
	unsigned char * ptr=(unsigned char *)Readarray.c_array();

	int index=0;
	unsigned char * ps1  = (unsigned char *)(&feedback_msg.LF_Encoder);

	* (ps1+3) = ptr[index++];
	* (ps1+2) = ptr[index++];
	* (ps1+1) = ptr[index++];
	* (ps1+0) = ptr[index++];

	//std::cout<<" LF_counter = " << LF_counter << std::endl;

	unsigned char * ps2  = (unsigned char *)(&feedback_msg.RF_Encoder);

	* (ps2+3) = ptr[index++];
	* (ps2+2) = ptr[index++];
	* (ps2+1) = ptr[index++];
	* (ps2+0) = ptr[index++];

	//std::cout<<" RF_counter = " <<RF_counter<<std::endl;

	unsigned char * psl1  = (unsigned char *)(&feedback_msg.LB_Encoder);

	* (psl1+3) = ptr[index++];
	* (psl1+2) = ptr[index++];
	* (psl1+1) = ptr[index++];
	* (psl1+0) = ptr[index++];

	//std::cout<<" LB_counter = " <<LB_counter<<std::endl;

	unsigned char * psR1  = (unsigned char *)(&feedback_msg.RB_Encoder);

	* (psR1+3) = ptr[index++];
	* (psR1+2) = ptr[index++];
	* (psR1+1) = ptr[index++];
	* (psR1+0) = ptr[index++];

	//std::cout<<" RB_counter = " << RB_counter << std::endl;
	feedback_msg.LFMotor_Status = Readarray[index++];   //Readarray[8];
	feedback_msg.RFMotor_Status = Readarray[index++];
	feedback_msg.LBMotor_Status = Readarray[index++];   //Readarray[8];
	feedback_msg.RBMotor_Status = Readarray[index++];

	char LFMotor_Error[4],RFMotor_Error[4],LBMotor_Error[4],RBMotor_Error[4];
	feedback_msg.LFMotor_Error.resize(4);
	feedback_msg.RFMotor_Error.resize(4);
	feedback_msg.LBMotor_Error.resize(4);
	feedback_msg.RBMotor_Error.resize(4);

//	unsigned char * ps3  = (unsigned char *)(&feedback_msg.LFMotor_Error);
	//*(ps3+1) = Readarray[index++];
	//*(ps3+0) = Readarray[index++];
	char ps_tmp = Readarray[index];
	LFMotor_Error[1] = hex2char(ps_tmp & 0x0f);
	LFMotor_Error[0] = hex2char((ps_tmp >> 4) & 0x0f);
	index++;
	ps_tmp = Readarray[index];
	LFMotor_Error[3] = hex2char(ps_tmp & 0x0f);
	LFMotor_Error[2] = hex2char((ps_tmp >> 4) & 0x0f);
	index++;
	feedback_msg.LFMotor_Error.assign(LFMotor_Error,4);

//	unsigned char * ps4  = (unsigned char *)(&feedback_msg.RFMotor_Error);

//	* (ps4+1) = Readarray[index++];
//	* (ps4+0) = Readarray[index++];
	ps_tmp = Readarray[index];
	RFMotor_Error[1] = hex2char(ps_tmp);
	RFMotor_Error[0] = hex2char((ps_tmp >> 4) & 0x0f);
	index++;
	ps_tmp = Readarray[index];
	RFMotor_Error[3] = hex2char(ps_tmp & 0x0f);
	RFMotor_Error[2] = hex2char((ps_tmp >> 4) & 0x0f);
	index++;
	feedback_msg.RFMotor_Error.assign(RFMotor_Error,4);

//	unsigned char * ps5  = (unsigned char *)(&feedback_msg.LBMotor_Error);

//	* (ps5+1) = Readarray[index++];
//	* (ps5+0) = Readarray[index++];
	ps_tmp = Readarray[index];
	LBMotor_Error[1] = hex2char(ps_tmp & 0x0f);
	LBMotor_Error[0] = hex2char((ps_tmp >> 4) & 0x0f);
	index++;
	ps_tmp = Readarray[index];
	LBMotor_Error[3] = hex2char(ps_tmp & 0x0f);
	LBMotor_Error[2] = hex2char((ps_tmp >> 4) & 0x0f);
	index++;
	feedback_msg.LBMotor_Error.assign(LBMotor_Error,4);

//	unsigned char * ps6  = (unsigned char *)(&feedback_msg.RBMotor_Error);

//	* (ps6+1) = Readarray[index++];
//	* (ps6+0) = Readarray[index++];
	ps_tmp = Readarray[index];
	RBMotor_Error[1] = hex2char(ps_tmp & 0x0f);
	RBMotor_Error[0] = hex2char((ps_tmp >> 4) & 0x0f);
	index++;
	ps_tmp = Readarray[index];
	RBMotor_Error[3] = hex2char(ps_tmp & 0x0f);
	RBMotor_Error[2] = hex2char((ps_tmp >> 4) & 0x0f);
	index++;
	feedback_msg.RBMotor_Error.assign(RBMotor_Error,4);


	//�����¶�

	feedback_msg.TEMP_Chassis = Readarray[index++];
	feedback_msg.Battery_Power = Readarray[index++];

	unsigned short int Ivot=0;
	unsigned char * ps4  = (unsigned char *)(&Ivot);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];

	feedback_msg.Battery_Volt = (Ivot*1.0)/100.0;

	short int Icurent=0;
	ps4  = (unsigned char *)(&Icurent);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];

	feedback_msg.Battery_Current =(Icurent*1.0)/100.0;

	feedback_msg.Battery_Temp = Readarray[index++];

	//����ֵ
	ps4  = (unsigned char *)(&feedback_msg.Obt1_Dis);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];
	ps4  = (unsigned char *)(&feedback_msg.Obt2_Dis);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];
	ps4  = (unsigned char *)(&feedback_msg.Obt3_Dis);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];

	unsigned char obstacle_status = Readarray[index++];
	feedback_msg.FC_Alarm = (obstacle_status) & 0x01; //<< 7 >> 7;
	feedback_msg.OA_Alarm = (obstacle_status >> 2) & 0x01;
	feedback_msg.ES_Alarm = (obstacle_status >> 4) & 0x01;
	feedback_msg.FP_Alarm = (obstacle_status >> 5) & 0x01;
	feedback_msg.BC_Status =(obstacle_status >> 6) & 0x01;

	unsigned char online_status = Readarray[index++];
	feedback_msg.LFW_Status = (online_status)& 0x01;
	feedback_msg.RFW_Status = (online_status >> 1)& 0x01;
	feedback_msg.LBW_Status = (online_status >> 2)& 0x01;
	feedback_msg.RBW_Status = (online_status >> 3)& 0x01;
	feedback_msg.OBT_Status = (online_status >> 4)& 0x01;
	feedback_msg.BMS_Status = (online_status >> 5)& 0x01;
	feedback_msg.FP1_Status = (online_status >> 6)& 0x01;
	feedback_msg.FP2_Status = (online_status >> 7)& 0x01;

	//short int IDrop1_dis = 0;
	ps4  = (unsigned char *)(&feedback_msg.FP1_Dis);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];

	ps4  = (unsigned char *)(&feedback_msg.FP2_Dis);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];

	ps4  = (unsigned char *)(&feedback_msg.LF_Torque);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];

	ps4  = (unsigned char *)(&feedback_msg.RF_Torque);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];

	ps4  = (unsigned char *)(&feedback_msg.LB_Torque);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];

	ps4  = (unsigned char *)(&feedback_msg.RB_Torque);
	* (ps4+1) = Readarray[index++];
	* (ps4+0) = Readarray[index++];

	feedback_msg_pub.publish(feedback_msg);
}
