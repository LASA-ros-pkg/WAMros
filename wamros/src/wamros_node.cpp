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
#include <wam_msgs/HomogeneousMatrix.h>
#include "WamNode.hpp"

// Note: A previous version of this code had some callback timing
// stuff. The frequency of callbacks to topics can be determined via
// other methods in ROS and the timings are not needed to determine
// the idle or running state of the robot (we use btclient states for
// that).

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

//Callbacks - real quick, major processing done elsewhere


//////////////////////////////////////////////////////////////////////////////////////////////
//This service takes no arguments and will move the WAM into what it thinks is it's home position.
bool goHomeSRV(std_srvs::Empty::Request   &req, std_srvs::Empty::Response &res )
{
  MyWam->goHome();
  return true;
}

bool activeSRV(std_srvs::Empty::Request   &req, std_srvs::Empty::Response &res )
{
  MyWam->active();
  return true;
}
bool idleSRV(std_srvs::Empty::Request   &req, std_srvs::Empty::Response &res )
{
  MyWam->idle();
  return true;
}

//////////////////////////////////////////////////////////////////////////////////
//This service will (de)activate the single joints.
bool joint_activeSRV(wam_msgs::ActivePassive::Request  &req,
	       wam_msgs::ActivePassive::Response &res )
{
  if(req.active.size() != 7)
    return false;
  for(int d=0;d<7;d++)
    MyWam->joint_active[d]=req.active[d];
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
  if (MyWam->state() == POS and MyWam->controller() == JOINT){ // POS mode and JSC loaded!
    MyWam->setTargetJoints(&msg->radians[0]);
  }
}

void HMref_callback(const wam_msgs::CartesianCoordinatesConstPtr &msg){
  if (MyWam->state() == POS and MyWam->controller() == CARTESIAN){

    ROS_DEBUG("Calling HMref_callback");
    ROS_DEBUG("position received: %f %f %f", msg->position[0], msg->position[1], msg->position[2]);
    ROS_DEBUG("orient received: %f %f %f", msg->euler[0], msg->euler[1], msg->euler[2]);

    MyWam->setTargetPosition(&msg->position[0], &msg->euler[0]);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//Main

int main(int argc, char **argv)
{

  MyWam = new WamNode();

  //Start motors limp
  for(int d=0;d<7;d++)
    MyWam->joint_active[d]=false;

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
  ros::Publisher wam_HMpos_pub = n.advertise<wam_msgs::HomogeneousMatrix>("hmpos", 1);
  ros::Publisher wam_HMref_pub = n.advertise<wam_msgs::HomogeneousMatrix>("hmref", 1);


  ros::Publisher wam_JA_pub = n.advertise<wam_msgs::JointAngles>("joints_sensed", 1);
  ros::Publisher wam_ST_pub = n.advertise<wam_msgs::WamStatus>("status",1);

  ros::Subscriber wam_JA_sub = n.subscribe("joints_command", 200, Jref_callback);
  ros::Subscriber wam_CC_sub = n.subscribe("cartesian_command", 200, HMref_callback);

  ros::ServiceServer switch_mode_srv = n.advertiseService("switchMode", switchModeSRV);

  ros::ServiceServer active_service = n.advertiseService("active_passive", joint_activeSRV);
  ros::ServiceServer movetopos_srv = n.advertiseService("moveToPos",moveToSRV);
  ros::ServiceServer goHome_srv = n.advertiseService("goHome",goHomeSRV);
  ros::ServiceServer movetocart_srv = n.advertiseService("moveToCart", moveToCartSRV);

  ros::ServiceServer idle_srv = n.advertiseService("idle",idleSRV);
  ros::ServiceServer active_srv = n.advertiseService("active",activeSRV);

  wam_msgs::JointAngles currentJA;
  currentJA.radians.resize(7);

  wam_msgs::CartesianCoordinates currentCC;

  wam_msgs::HomogeneousMatrix currentHM;
  wam_msgs::HomogeneousMatrix currentHMref;

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

    double * hm = MyWam->getHomogeneousMatrix();
    for (int i=0;i<16;i++){
      currentHM.element[i] = hm[i];
    }

    hm = MyWam->getCartesianCommand();
    for (int i = 0; i< 16; i++){
      currentHMref.element[i] = hm[i];
    }

    Mode cntrl = MyWam->controller();
    State stat = MyWam->state();

    currentStatus.status = (int) stat;
    currentStatus.controller = (int) cntrl;

    // Cartesian coordinate info
    wam_CC_pub.publish(currentCC);
    wam_HMpos_pub.publish(currentHM);
    wam_HMref_pub.publish(currentHMref);

    wam_JA_pub.publish(currentJA);
    wam_ST_pub.publish(currentStatus);

    ros::spinOnce();
    loop_rate.sleep();
  }

  //Cleanup
  MyWam->cleanup();
}
