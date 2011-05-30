#include "btwam.h" 
#include "WamNode.hpp"

//Stuff we need
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>
#include <curses.h>
#include <fstream>

WamNode * TheWam = NULL; // store singleton wam-wrapper object 

WamNode::WamNode()
{
  wam = NULL;
  movingToPos = false;

  if(TheWam != NULL) 
    {
      // we already created a WamNode, this is bad .. 
      // should never happen .. 
      printf("Can't create two WamNode .. you have only one wam right ?? \n");
      exit(1);
    }
  TheWam = this;
  btrt_mutex_create(&target_buffer_mutex);
}

//Realtime Thread for CAN communications
//Errors use fprintf which is not-realtime, because...they're errors
// we should stick to c - style function calls for threads .. 

void rt_thread(void *thd_){
  
  int err;
  
  btrt_thread_struct * thd = (btrt_thread_struct *) thd_;  
  char *conf = (char *) thd->data;

  //Probe and initialize the robot actuators
  err = InitializeSystem();
  if(err) {
    fprintf(stderr,"InitializeSystem returned err = %d", err);
    exit(1);
  }
	
  //Initialize and get a handle to the robot on the first bus
  printf("opening config file in %s \n",conf);
  if((TheWam-> wam = OpenWAM(conf, 0)) == NULL){
    fprintf(stderr, "OpenWAM failed");
    exit(1);
	}
	
  /* setSafetyLimits(bus, joint rad/s, tip m/s, elbow m/s);
   * For now, the joint and tip velocities are ignored and
   * the elbow velocity provided is used for all three limits.
   */
  setSafetyLimits(0, 4, 4, 4);
  /* Set the puck torque safety limits (TL1 = Warning, TL2 = Critical).
   * Note: The pucks are limited internally to 3441 (see 'MT' in btsystem.c) 
   * Note: btsystem.c bounds the outbound torque to 8191, so entering a
   * value of 9000 for TL2 would tell the safety system to never register a 
   * critical fault.
   */
  setProperty(0, SAFETY_MODULE, TL2, FALSE, 8000);
  setProperty(0, SAFETY_MODULE, TL1, FALSE, 5000);
  
  // We're inited
  TheWam->startDone = TRUE;
  /* Spin until we are told to exit */
  while (!btrt_thread_done(thd)){
    usleep(10000);
  }   
  /* Remove this thread from the realtime scheduler */
  btrt_thread_exit(thd);
}

void WamNode::init(std::string &conf)
{
  int   err;        // Generic error variable for function calls
  int   busCount;   // Number of WAMs defined in the configuration file
  /* Allow hard real time process scheduling for non-root users */
  mlockall(MCL_CURRENT | MCL_FUTURE);
  /* Xenomai non-root scheduling is coming soon! */
  /* Initialize syslog */
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  atexit(closelog);
	
  /* Read the WAM configuration file */
  TheWam->wamconf = conf;
  err = ReadSystemFromConfig((char *) TheWam->wamconf.c_str(), &busCount);
  if(err) {
    fprintf(stderr, "ReadSystemFromConfig returned err = %d", err);
    exit(1);
  }
 
  //User checks WAM
  printf("Make sure:\n");
  printf("1) WAM cables are secure\n");
  printf("2) WAM is on\n");
  printf("3) Estops are released\n");
  printf("4) Shift-Idle is active (LED=0)\n");
  printf("5) WAM is home (check wrist)\n");
  printf("Then press enter:");
  
  getchar();

  /* Spin off the RT task to set up the CAN Bus */
  startDone = FALSE;

  rt_thd = new_btrt_thread();
  wam_thd = new_btrt_thread();

  // firing a thread with RT priority 
  btrt_thread_create(rt_thd,"rtt", 45, (void*) rt_thread, (void *) TheWam->wamconf.c_str()); 

  while(!startDone)
    usleep(10000);
  
  registerWAMcallback(wam, (void*)WAMcallback);
  /* Spin off the WAM control loop */
  wam_thd->period = Ts; // Control loop period in seconds
  
  // this function ( WAMControlThread ) is not  ours .. it's barret's control
  // loop... 
  btrt_thread_create(wam_thd, "ctrl", 90, (void*)WAMControlThread, (void*)wam);
  //User activates WAM
  printf("Press Shift+Activate then press <Enter>:");

  getchar();

  clear();

  // to get the mutex lock, we need to become a xenomai task
  // this will transform the main thread in xenomai RT task, with priority 0
  // since there is no way to unshadow, it will stay as is. 
  // another solution would be to launch a rt_task to do the job.. 

  int stat_task = rt_task_shadow(NULL, "joint_writer", 0, 0);


  printf("To exit, press Shift-Idle on pendant, then hit Ctrl-C\n\n");

  /* Set gravity scale to 1.0g */
  SetGravityUsingCalibrated(wam,1);
  SetGravityComp(wam, 1.0); 

  //Start where we are and locked
  for(int d=0;d<7;d++){
    wam->Jref->q[d] = wam->Jpos->q[d]; // reading actual joint position
    setTargetJoints(wam->Jpos->q);
    active[d]=true;		
  }

  //Do all the wam stuff before ROS kicks in
  SetJointSpace(wam);//Necessary?
  MoveSetup(wam, 0.5, 0.5 );
}


/////////////////////////////////////////////////////////////////////////////////////////
int WAMcallback(struct btwam_struct *m_wam)
{
  double targetjoints[7];

  
  // reading joints angle/torques and store them 
  // so that by default we reuse the last sent read joints, -> not moving
  TheWam->RTgetTargetJoints(targetjoints); 
  
  for(int d=0;d<7;d++){

    TheWam->m_Jtrq[d] = m_wam->Jtrq->q[d]; //Record what control would say

    if(! TheWam->active[d]){ //We're passive
			m_wam->Jtrq->q[d]=0;    //no torque (other than gravity comp)
			targetjoints[d] = m_wam->Jpos->q[d]; // and stay there ! 
		}
  }

  //We have our own movingToPos which has to be reset after the WAM has finished moving.
  // we also don't want to send joint targets while the wam is moving during 
  // a MoveWam  call ? 

  if(MoveIsDone(m_wam) || !TheWam->movingToPos)	//Work around barretts programming style
    {
      TheWam->movingToPos = false;    
			
      for(int d=0;d<7;d++)
				{
					m_wam->Jref->q[d] = targetjoints[d];
				}
			
    }
  
  TheWam->RTupdateTargetJoints(targetjoints); // update buffers 
  return 1;
}


/* Exit the realtime threads and close the system */
void WamNode::cleanup()
{
  // shutdown realtime loop. 
  wam_thd->done = TRUE;
  usleep(10000);   
  CloseSystem();
  rt_thd->done = TRUE;
  
  // cleaning up the mutex ?? (btos.h has no mutex_delete thing.. )
  //Remove wam.conf.flt/tmp files
  system("rm -f wam.conf.flt wam.conf.tmp"); // eark .. 
  // FIXME : do a proper thread_join ?? 
  
}

void WamNode::goTo(const double * pos, bool wait)
{ 
  for(int d=0;d<7;d++)
    {	
      moveToPos[d] = pos[d];	
    }
  setTargetJoints(moveToPos);
  movingToPos = true;
  vect_n * vector2 = new_vn(wam->dof );

	
  const_vn( vector2,moveToPos[0],moveToPos[1],moveToPos[2],moveToPos[3],moveToPos[4],moveToPos[5],moveToPos[6] );

  for(int d=0;d<7;d++)
    active[d]=true;

  MoveWAM(wam, vector2);
  destroy_vn(&vector2);
 
	// waiting 'til we reach the posture
  if(wait) 
		while(!MoveIsDone(wam))
			usleep(10000);
}

/////////////////////////////////////////////////////////////////////////////////////////
//Stretch after waking up. Make sure not to get out of bed with the wrong leg first.
void WamNode::initMoves()
{

  moveToPos[0] = 0.3;
  moveToPos[1] = -1.7;
  moveToPos[2] = 0.3;
  moveToPos[3] = 2.84;
  moveToPos[4] = 0.3;
  moveToPos[5] = 0.3;
  moveToPos[6] = 0.3;
	
  goTo(moveToPos);

	//This should be home
  moveToPos[0] = 0.0;
  moveToPos[1] = -2.0;
  moveToPos[2] = 0.0;
  moveToPos[3] = 3.14;
  moveToPos[4] = 0;
  moveToPos[5] = 0;
  moveToPos[6] = 0.0;

  goTo(moveToPos);
}

double * WamNode::getJoints()
{
  return wam->Jpos->q;
}

double * WamNode::getJointsCommand()
{
  return wam->Jref->q;
}

double * WamNode::getTorques()
{
  return wam->Jtrq->q;
}

double * WamNode::getTotalTorques()
{
  return wam->Ttrq->q;
}

double * WamNode::getGravityTorques()
{
  return wam->Gtrq->q;
}


double * WamNode::getMotorTorques()
{
  return wam->Mtrq->q;
}

double * WamNode::getMotorAngles()
{
  return wam->Mpos->q;
}

double * WamNode::getCartesian()
{
  return wam->Cpos->q;
}

bool WamNode::RTupdateTargetJoints(const double * target)
{
  for(int i=0;i<7;i++)
    mPreviousTargetJoints[i] = target[i];
  
  int stat = rt_mutex_acquire(&target_buffer_mutex, TM_NONBLOCK);
  
  if(stat==0) // we got the mutex !! 
    {
      for(int i=0;i<7;i++)
				mTargetJoints[i] = mPreviousTargetJoints[i]; 
      rt_mutex_release(&target_buffer_mutex);
    }
  return stat;
}
  
bool WamNode::RTgetTargetJoints(double * target)
{
  // trying to get the mutex, but don't block if it's not 
  // directly available, then return last read target position..
  int stat = rt_mutex_acquire(&target_buffer_mutex, TM_NONBLOCK);
  
  if(stat==0) // we got the mutex !! 
    {
      for(int i=0;i<7;i++)
				target[i] = mPreviousTargetJoints[i] = mTargetJoints[i];
      rt_mutex_release(&target_buffer_mutex);
    }
  else // cant get instant access to the buffer
    {
      for(int i=0;i<7;i++)
				target[i] = mPreviousTargetJoints[i];
    }
  return stat != 0;
}


void WamNode::setTargetJoints(const double * target) 
{
  int stat = rt_mutex_acquire(&target_buffer_mutex,TM_INFINITE); // blocks until mutex is ok

  if(stat==0)
    {
      for (int i = 0; i < 7; i++)
				{
					mTargetJoints[i] = target[i];
				}
      rt_mutex_release(&target_buffer_mutex);
    }
  // else -> epic fail .. 
} 

