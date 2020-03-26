#include <map>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
//#include <diagnostic_updater/diagnostic_updater.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "model_com/interface_can.h"
#include "model_trx.h"
#include "model_kinco.h"


using namespace Chassis_Driver;

template<class T>
class Chassis_Control:public T
{
public:
	Chassis_Control();
	~Chassis_Control();

//	bool Init(){
//		((T*)(this))->Init_model();
//		return true;
//	};


private:
	//ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "3bot_chassis_driver");
  ros::NodeHandle nh;

#ifdef CHASSIS_STYLE
	  #define MODEL_CHASSIS Model_Kinco
#else
	  #define MODEL_CHASSIS Model_Trx
#endif
  Chassis_Control<MODEL_CHASSIS> cm;
  ros::spin();
  // To quote Morgan, Hooray!
  return(0);
}


template<class T>
Chassis_Control<T>::Chassis_Control():
	private_nh_("~")
{

	//((T*)(this))->Init_model();
}

template<class T>
Chassis_Control<T>::~Chassis_Control()
{

}







