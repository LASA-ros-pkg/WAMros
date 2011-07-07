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
#include <wam_msgs/MoveToCart.h>
#include <std_srvs/Empty.h>
#include <wam_msgs/SwitchModes.h>
#include <wam_msgs/WamStatus.h>
#include <wam_msgs/CartesianCoordinates.h>
#include <wam_msgs/CartesianTargets.h>
#include <wam_msgs/HomogeneousMatrix.h>
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
  MyWam->goTo(target,false); // don't wait , don't block ROS..
  return true;
}

bool switchModeSRV(wam_msgs::SwitchModes::Request &req, wam_msgs::SwitchModes::Response &res ){
  switch((int) req.mode){
  case 0:
    ROS_INFO("Switching to JOINT space");
    MyWam->switchSpace(JOINT);
    break;
  case 1:
    ROS_INFO("Switching to CARTESIAN space");
    MyWam->switchSpace(CARTESIAN);
    break;
  }

  ROS_INFO("isZerod: %d", MyWam->getZeroed());
  ROS_INFO("Active SC Code: %d", MyWam->getActiveSC());
  return true;
}

bool moveToCartSRV(wam_msgs::MoveToCart::Request &req, wam_msgs::MoveToCart::Response &res )
{
  // not sure if this is necessary -- sizes are defined in msgs
  if (req.cart.position.size() != 3 || req.cart.euler.size() != 3)
    return false;

  double *pos = &req.cart.position[0];
  double *orient = &req.cart.euler[0];
  MyWam->goToCart(pos, orient, false);
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//Write where we want to go into desired m_Jref

void Jref_callback(const wam_msgs::JointAnglesConstPtr& msg)
{
  if(!MyWam->movingToPosCart and !MyWam->movingToPos and msg->radians.size()==7) // checking if we aren't in goto mode
    {
       struct timespec tv;
       // RT timer
       // on a standart linux this has a micro second resolution.

       clock_gettime(CLOCK_MONOTONIC,&tv);
       last_command_us =  tv.tv_nsec / 1000 + tv.tv_sec * 1000000;
       MyWam->setTargetJoints(&msg->radians[0]);
    }
}

void HMref_callback(const wam_msgs::CartesianTargetsConstPtr &msg){
  double *tp,*hmref, *hmpos;

  if (!MyWam->movingToPosCart and !MyWam->movingToPos and msg->pos.size() == 3){
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    last_command_us = tv.tv_nsec / 1000 + tv.tv_sec * 1000000;

    ROS_DEBUG("Calling HMref_callback");
    ROS_DEBUG("position received: %f %f %f", msg->pos[0], msg->pos[1], msg->pos[2]);

    MyWam->setTargetPosition(&msg->pos[0]);

    hmref = MyWam->getCartesianCommand();
    hmpos = MyWam->getHomogeneousMatrix();
    tp = MyWam->getCartesianTargets();
    ROS_DEBUG("target position: %f %f %f", tp[0], tp[1], tp[2]);
    ROS_DEBUG("hmref position: %f %f %f", hmref[3], hmref[7], hmref[11]);
    ROS_DEBUG("hmpos position: %f %f %f", hmpos[3], hmpos[7], hmpos[11]);
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

  ros::Publisher wam_CC_pub = n.advertise<wam_msgs::CartesianCoordinates>("cartesian_coordinates", 1);
  ros::Publisher wam_HM_pub = n.advertise<wam_msgs::HomogeneousMatrix>("homogeneous_matrix", 1);
  ros::Publisher wam_CT_pub = n.advertise<wam_msgs::CartesianTargets>("cartesian_targets", 1);

  ros::Publisher wam_JA_pub = n.advertise<wam_msgs::JointAngles>("joints_sensed", 1);
  ros::Publisher wam_ST_pub = n.advertise<wam_msgs::WamStatus>("status",1);

  ros::Subscriber wam_JA_sub = n.subscribe("joints_command", 200, Jref_callback);
  ros::Subscriber wam_CC_sub = n.subscribe("cartesian_command", 200, HMref_callback);

  ros::ServiceServer switch_mode_srv = n.advertiseService("switchMode", switchModeSRV);

  ros::ServiceServer active_service = n.advertiseService("active_passive", activeSRV);
  ros::ServiceServer movetopos_srv = n.advertiseService("moveToPos",moveToSRV);
  ros::ServiceServer goHome_srv = n.advertiseService("goHome",goHomeSRV);
  ros::ServiceServer movetocart_srv = n.advertiseService("moveToCart", moveToCartSRV);

  wam_msgs::JointAngles currentJA;
  currentJA.radians.resize(7);

  wam_msgs::CartesianCoordinates currentCC;
  wam_msgs::CartesianTargets currentCT;
  wam_msgs::HomogeneousMatrix currentHM;

  wam_msgs::WamStatus currentStatus;

  while(ros::ok()){ //Catching ctrlC is done by ROS

    double * joints = MyWam->getJoints();
    for(int d=0;d<7;d++){
      currentJA.radians[d] = joints[d];
    }

    // position is in meters
    double * cp = MyWam->getCartesianPosition();
    for (int i=0;i<3;i++){
      currentCC.position[i] = cp[i];
    }


    double * co = MyWam->getCartesianOrientation();
    for (int i=0;i<3;i++){
      currentCC.euler[i] = co[i];
    }

    double * ct = MyWam->getCartesianTargets();
    for (int i = 0; i<3; i++){
      currentCT.pos[i] = ct[i];
    }

    double * hm = MyWam->getHomogeneousMatrix();
    for (int i=0;i<16;i++){
      currentHM.element[i] = hm[i];
    }

    if( MyWam->movingToPos || MyWam->movingToPosCart)
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

    // Cartesian coordinate info
    wam_CC_pub.publish(currentCC);
    wam_HM_pub.publish(currentHM);
    wam_CT_pub.publish(currentCT);

    wam_JA_pub.publish(currentJA);
    wam_ST_pub.publish(currentStatus);

    ros::spinOnce();
    loop_rate.sleep();
  }

  //Cleanup
  MyWam->cleanup();
}
