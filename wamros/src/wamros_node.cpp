/* Connect the Barret WAM to ROS
   
   Init WAM
   Init ROS
	 Lock individual joints with service   
	 Spit sensed joint angles to topic
   Take in commanded joint angles and follow
*/

//Stuff we need
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>

//ROS stuff - all messages and services
#include <ros/ros.h>
#include <wam_msgs/JointAngles.h>
#include <wam_msgs/ActivePassive.h>
#include <wam_msgs/MoveToPos.h>
//#include <wam_msgs/GoHome.h>
#include <std_srvs/Empty.h>
#include <wam_msgs/WamStatus.h>

#include "WamNode.hpp"

//Useful macros
#define MIN(x,y) (x<y ? x : y)
#define MAX(x,y) (x>y ? x : y)
#define SIGN(x) (x<0 ? -1 : 1)
#define ABS(x) (x<0 ? -x : x)
#define FOR(i,n) for(int i=0; i<n; i++)

WamNode * MyWam = NULL;

// Wam specific parameters (see start.sh for how to set)
std::string wamconf; // Store the conf file here.
bool doinit = false; // Do init motion on startup.

unsigned long last_command_us=0; // when was last joint command
				 // received (in microsecond since
				 // beginning) , 4 bytes integer =>
				 // more that 4.10^3 second => more
				 // than one hour

//Callbacks - real quick, major processing done elsewhere


//////////////////////////////////////////////////////////////////////////////////////////////
//This service takes no arguments and will move the WAM into what it thinks is it's home position.
bool goHomeSRV(std_srvs::Empty::Request   &req, std_srvs::Empty::Response &res )
{
  MyWam->goHome();
  return true;
}
//////////////////////////////////////////////////////////////////////////////////
//This service will (de)activate the single joints.
bool activeSRV(wam_msgs::ActivePassive::Request  &req,
	       wam_msgs::ActivePassive::Response &res )
{
  if(req.active.size() != 7)
    return false;
  for(int d=0;d<7;d++)
    MyWam->active[d]=req.active[d];
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//Will move the WAM to the joint angles provided in the request.

bool moveToSRV(wam_msgs::MoveToPos::Request  &req, wam_msgs::MoveToPos::Response &res )
{
  if(req.pos.radians.size()!=7)
    return false;
  double * target = &req.pos.radians[0];
  MyWam->goTo(target,false); // don't wait , don't block ROS .. 
  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////
//Write where we want to go into desired m_Jref

void Jref_callback(const wam_msgs::JointAnglesConstPtr& msg)
{
  if(!MyWam->movingToPos and msg->radians.size()==7) // checking if we aren't in goto mode 
    {
       struct timespec tv;
       // RT timer
       // on a standart linux this has a micro second resolution. 

       clock_gettime(CLOCK_MONOTONIC,&tv); 
       last_command_us =  tv.tv_nsec / 1000 + tv.tv_sec * 1000000;

       for(int d=0;d<7;d++)
	 {
	   MyWam->setTargetJoints(&msg->radians[0]); 
	 }
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
//Main

int main(int argc, char **argv)
{

  MyWam = new WamNode();

  //Start motors limp
  for(int d=0;d<7;d++)
    MyWam->active[d]=false;

  //Start up ROS 
  ros::init(argc, argv, "wamros_node");
  ros::NodeHandle n("wam"); // adding the wam namespace for topics/services
  ros::Rate loop_rate(500);
  
  //Start up WAM
  ros::param::get("~wamconf", wamconf);
  ros::param::get("~doinit", doinit); 
  printf("Using config file: %s\n", wamconf.c_str());
  MyWam->init(wamconf);

  double * init_joints = MyWam->getJoints();
  if (doinit)
    MyWam->initMoves();  	// do the nice motion at startup.

  ros::Publisher wam_JA_pub = n.advertise<wam_msgs::JointAngles>("joints_sensed", 1);
  ros::Publisher wam_ST_pub = n.advertise<wam_msgs::WamStatus>("status",1);

  ros::Subscriber wam_JA_sub = n.subscribe("joints_command", 200, Jref_callback);

  ros::ServiceServer active_service = n.advertiseService("active_passive", activeSRV);
  ros::ServiceServer movetopos_srv = n.advertiseService("moveToPos",moveToSRV);
  ros::ServiceServer goHome_srv = n.advertiseService("goHome",goHomeSRV);

  wam_msgs::JointAngles currentJA;
  currentJA.radians.resize(7);
  wam_msgs::WamStatus currentStatus;

  while(ros::ok()){ //Catching ctrlC is done by ROS

    double * joints = MyWam->getJoints();
    for(int d=0;d<7;d++){
      currentJA.radians[d] = joints[d];
    }
    
    if( MyWam->movingToPos)
      currentStatus.status = wam_msgs::WamStatus::MOVING;
    else 
      {
				struct timespec tv;
				clock_gettime(CLOCK_MONOTONIC,&tv); 
				unsigned long current_time_ = tv.tv_nsec / 1000 + tv.tv_sec * 1000000; 
				if( (current_time_ - last_command_us  ) < 1e5) // 100 ms idle time ? 
					currentStatus.status = wam_msgs::WamStatus::COMMAND;
				else
					currentStatus.status = wam_msgs::WamStatus::IDLE;
      }

    wam_JA_pub.publish(currentJA);
    wam_ST_pub.publish(currentStatus);

    ros::spinOnce();		
    loop_rate.sleep();		
  }

  //Cleanup
  MyWam->cleanup();
}
