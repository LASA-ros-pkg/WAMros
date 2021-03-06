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

  callback_pos = new_mh();
  mTargetPos = new_mh();
  mPreviousTargetPos = new_mh();
  RXRYRZ = new_v3();
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

  //Do all the wam stuff before ROS kicks in
  TheWam->switchSpace(CARTESIAN);
}


/////////////////////////////////////////////////////////////////////////////////////////
int WAMcallback(struct btwam_struct *m_wam)
{
  double targetjoints[7];

  switch(TheWam->getCurrentMode()){

  case JOINT:

    // reading joints angle/torques and store them
    // so that by default we reuse the last sent read joints, -> not moving
    TheWam->RTgetTargetJoints(targetjoints);

    // This is per-joint idle magic. A better method of
    // initializing overall passive behavior is to call idle().

    for(int d=0;d<7;d++){
      TheWam->m_Jtrq[d] = m_wam->Jtrq->q[d]; //Record what control would say

      if(!TheWam->joint_active[d]){  //We're passive
	m_wam->Jtrq->q[d]=0;    //no torque (other than gravity comp)
	targetjoints[d] = m_wam->Jpos->q[d]; // and stay there !
      }
    }

    // If any of the wam move services are called then Jref should not
    // be set during callbacks.

    if(MoveIsDone(m_wam) and TheWam->state() == POS){
      for(int d=0;d<7;d++){
	m_wam->Jref->q[d] = targetjoints[d];
      }
    } else {
      // update target joints since we are moving using MoveWam or IDLE
      for (int d = 0; d<7; d++)
	targetjoints[d] = m_wam->Jpos->q[d];
    }

    TheWam->RTupdateTargetJoints(targetjoints); // update buffers

    break;

  case CARTESIAN:

    TheWam->RTgetTargetPosition(TheWam->callback_pos);

    if(MoveIsDone(m_wam) and TheWam->state() == POS){

      // update position and orientation in CARTESIAN mode
      set_vn((vect_n *)m_wam->HMref, (vect_n *)TheWam->callback_pos);

    } else {
      // The wam is moving using MoveWAM or IDLE.
      set_vn((vect_n *) TheWam->callback_pos, (vect_n *) m_wam->HMpos);
    }

    // update buffers
    TheWam->RTupdateTargetPosition(TheWam->callback_pos);
    break;

  default:
    // err...
    break;
  }

  return 0;
}


/* Exit the realtime threads and close the system */
void WamNode::cleanup()
{
  // shutdown realtime loop.
  wam_thd->done = TRUE;
  usleep(10000);
  CloseSystem();
  rt_thd->done = TRUE;

  //Remove wam.conf.flt/tmp files
  system("rm -f wam.conf.flt wam.conf.tmp"); // eark ..

  // Free the vect_3 holding orientation.
  destroy_vn((vect_n **) &RXRYRZ);
  destroy_vn((vect_n **) &mTargetPos);
  destroy_vn((vect_n **) &mPreviousTargetPos);
  destroy_vn((vect_n **) &callback_pos);

}

void WamNode::hm2POS(const matr_h *hm, double *pos, double *orient){
  vect_3 *Rxyz;
  Rxyz = new_v3();
  RtoXYZf_m3((matr_3*)hm, Rxyz);

  orient[0] = Rxyz->q[0];
  orient[1] = Rxyz->q[1];
  orient[2] = Rxyz->q[2];

  pos[0] = ELEM(hm, 0, 3);
  pos[1] = ELEM(hm, 1, 3);
  pos[2] = ELEM(hm, 2, 3);

  destroy_vn((vect_n **) &Rxyz);
}

void WamNode::pos2HM(const double *pos, const double *orient, matr_h *hm){
  vect_3 *Rxyz;
  Rxyz = new_v3();

  const_v3(Rxyz, orient[0], orient[1], orient[2]);
  XYZftoR_m3((matr_3*)hm, Rxyz); // Convert from Rxyz to R[3x3]

  ELEM(hm, 0, 3) = pos[0]; // Insert the X position
  ELEM(hm, 1, 3) = pos[1]; // Insert the Y position
  ELEM(hm, 2, 3) = pos[2]; // Insert the Z position

  destroy_vn((vect_n **) &Rxyz);
}

void WamNode::goToCart(const double * pos, const double * orient, bool wait)
{

  matr_h *matrix;

  movingToPosCart = true;

  // Convert from X, Y, Z, Rx, Ry, Rz to a homogeneous matrix
  matrix = new_mh();

  TheWam->pos2HM(pos,orient,matrix);

  TheWam->switchSpace(CARTESIAN);

  // In Cartesian space, MoveWAM() expects the full homogeneous matrix in vector format
  MoveSetup(wam, 0.5, 0.5 );
  MoveWAM(wam, (vect_n*)matrix);

  if (wait)
    while (!MoveIsDone(wam)) usleep(10000);

  destroy_mn((matr_mn **)&matrix);
}

void WamNode::goTo(const double * pos, bool wait)
{
  for(int d=0;d<7;d++)
    {
      moveToPos[d] = pos[d];
    }

  movingToPos = true;
  vect_n * vector2 = new_vn(wam->dof );

  const_vn( vector2,moveToPos[0],moveToPos[1],moveToPos[2],moveToPos[3],moveToPos[4],moveToPos[5],moveToPos[6] );

  for(int d=0;d<7;d++)
    joint_active[d]=true;

  TheWam->switchSpace(JOINT);

  MoveSetup(wam, 0.5, 0.5 );
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

  moveToPos[0] = 0.0010591760997235107;
  moveToPos[1] = 0.91468612901675161;
  moveToPos[2] = -0.16137578495734498;
  moveToPos[3] = 1.8221148811526526;
  moveToPos[4] = 0.28729767170550075;
  moveToPos[5] = 0.374190856558136;
  moveToPos[6] = -0.91668911420833599;

  goTo(moveToPos);

  // 0.6229972628252165, -0.056797047078617914, -0.015403360732277913, -2.5847001225425954, 0.025660708065062269, -3.0920239607278357

  double pos[3];
  double orient[3];

  pos[0] = 0.6229972628252165;
  pos[1] = -0.056797047078617914;
  pos[2] = -0.015403360732277913;
  orient[0] = -2.5847001225425954;
  orient[1] = 0.025660708065062269;
  orient[2] = -3.0920239607278357;
  goToCart(pos, orient, true);

}

Mode WamNode::getCurrentMode(){
  return mode;
}

int WamNode::getZeroed(){
  return wam->isZeroed;
}

int WamNode::getActiveSC(){

  if (wam->active_sc == &wam->Jsc){
    return 0;
  } else if (wam->active_sc == &wam->Csc) {
    return 1;
  } else {
    return 2; // huh?
  }

}

Mode WamNode::switchSpace(Mode nmode){

  // This will set the target joint or positions to the current joint
  // or positions. After the switch the controller will be
  // idle. MoveWam or activate will start the corresponding the
  // controller.

  switch(nmode){

  case CARTESIAN:

    setTargetPosition(wam->HMpos);

    SetCartesianSpace(wam);
    mode = CARTESIAN;
   break;

  case JOINT:

    setTargetJoints(wam->Jpos->q);

    SetJointSpace(wam);
    mode = JOINT;
    break;

  default:
    //err...
    break;
  }

  // Not sure if we need to call this again. Does not seem to hurt.
  return mode;
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

double * WamNode::getCartesianOrientation(){
  // From the btclient documentation:

  // NOTE: matr_3 is actually a 4x4 "homogeneous" matrix: [ r11 r12
  // r13 x ] [ r21 r22 r23 y ] [ r31 r32 r33 z ] [ 0 0 0 1 ] Many
  // functions only operate on the inner 3x3 "rotation" matrix.

  RtoXYZf_m3((matr_3*)wam->HMpos, RXRYRZ);
  return RXRYRZ->q;
}

double * WamNode::getHomogeneousMatrix(){
  return wam->HMpos->q;
}

double * WamNode::getCartesianPosition(){
  return wam->Cpos->q;
}

double * WamNode::getCartesianCommand(){
  return wam->HMref->q;
}

double * WamNode::getCartesianTargets(){
  return mTargetPos->q;
}

Mode WamNode::controller(){
  // what controller is active?

  if (wam->active_sc == &wam->Jsc){
    return JOINT;
  } else {
    return CARTESIAN;
  }
}

State WamNode::state(){
  // from btclient code
  // idle : 0
  // torque : 1
  // pos : 2
  // trj : 3
  return (State) getmode_bts(wam->active_sc);
}

void WamNode::active(){
  setmode_bts(wam->active_sc, SCMODE_POS);
}

void WamNode::idle(){
  setmode_bts(wam->active_sc, SCMODE_IDLE);
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

  if(stat==0){
    for (int i = 0; i < 7; i++){
      mTargetJoints[i] = target[i];
    }
    rt_mutex_release(&target_buffer_mutex);
  }
  // else -> epic fail ..
}


void WamNode::setTargetPosition(const matr_h *pos){
  int stat = rt_mutex_acquire(&target_buffer_mutex, TM_INFINITE);
  if (stat == 0){

    set_vn((vect_n *)mTargetPos, (vect_n *)pos);

    rt_mutex_release(&target_buffer_mutex);
  }
}


void WamNode::setTargetPosition(const double *pos){
  int stat = rt_mutex_acquire(&target_buffer_mutex, TM_INFINITE);
  if (stat == 0){

    ELEM(mTargetPos,0,3) = pos[0];
    ELEM(mTargetPos,1,3) = pos[1];
    ELEM(mTargetPos,2,3) = pos[2];

    rt_mutex_release(&target_buffer_mutex);
  }
}

void WamNode::setTargetPosition(const double *pos, const double *orient){
  matr_h *hm;
  hm = new_mh();
  pos2HM(pos, orient, hm);

  int stat = rt_mutex_acquire(&target_buffer_mutex, TM_INFINITE);
  if (stat == 0){

    set_vn((vect_n *) mTargetPos, (vect_n *) hm);

    rt_mutex_release(&target_buffer_mutex);
  }
  destroy_vn((vect_n **) &hm);
}

bool WamNode::RTupdateTargetPosition(const matr_h *pos){

  set_vn((vect_n *) mPreviousTargetPos, (vect_n *) pos);

  int stat = rt_mutex_acquire(&target_buffer_mutex, TM_NONBLOCK);


  if(stat==0){
    set_vn((vect_n *) mTargetPos, (vect_n *) mPreviousTargetPos);

    rt_mutex_release(&target_buffer_mutex);
  }
  return stat;
}

bool WamNode::RTgetTargetPosition(matr_h *pos){
  // trying to get the mutex, but don't block if it's not
  // directly available, then return last read target position..
  int stat = rt_mutex_acquire(&target_buffer_mutex, TM_NONBLOCK);

  if(stat==0) {// we got the mutex !!
    set_vn((vect_n *) mPreviousTargetPos, (vect_n *) mTargetPos);
    set_vn((vect_n *) pos, (vect_n *) mTargetPos);

   rt_mutex_release(&target_buffer_mutex);
  }
  else { // cant get instant access to the buffer
    set_vn((vect_n *) pos, (vect_n *) mPreviousTargetPos);
  }

  return stat != 0;
}
