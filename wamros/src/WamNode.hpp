
/**
 * a nice wrapper around Barret's wam_struct
 * for some OO cleaner code... 
 * This wrapper manages the realtime thread, the initialisation procedure
 * and provide a convenient way of sending joint targets
 * to robot. 
 *
`* Notes : this needs to be compiled with -DXENOMAI and all the xenomai stack 
 * since we are using a rt_mutex for the joint buffer. The init() method will actually 
 * transform the main thread (or the one calling this method ) into a xenomai task with 
 * priority 0 (non RT) to be able to lock the rt_mutex. All the other methods shall
 * be called by this same thread (or at least a xenomai thread .. )
`*
 * Florent D'halluin <florent.dhalluin@epfl.ch> 
 * Dan Grollman <daniel.grollman@epfl.ch>
 */



//The WAM stuff
#include <native/mutex.h>
#include <string>
#ifndef _BTWAM_H

struct wam_struct;
struct btwam_struct;
struct btrt_thread_struct;
struct vect_3;
struct matr_h;

#endif

//Should get from wam.conf
const double homePosition[] = { 0, -2, 0.0, 3.14, 0 , 0 , 0}; // "home" posture

// determine whether joint control is governed by the joint or cartesian topics
enum Mode { JOINT = 0, CARTESIAN };

#define Ts (0.002) // tick tock period .. 
#define WAM_X_CENTER (0.50)
#define WAM_Y_CENTER (0.00)
#define WAM_M_PER_PIX (0.001)
#define WAM_M_PER_TICK (0.0002)

class WamNode
{
protected :
  // these struct are wrapper arount pthread .. (see btos.h) 
  btrt_thread_struct * rt_thd;  // real time initialisation thread 
  btrt_thread_struct * wam_thd; // Barret control loop  

  RT_MUTEX target_buffer_mutex; // mutex for joints buffer

  double mTargetJoints[7]; // joints to be sent to robots 
  double mPreviousTargetJoints[7]; // joints sent to robots
                                   // this one is directly written by rt loop

  double mTargetPos[2];
  double mPreviousTargetPos[2];

  // nobody else should be able to update mode outside of switchSpace
  Mode mode;

public :

  // sad part : these should be protected / private , but 
  // needs to be accessible from thread and wam callback. 

  // TODO : write mutexed or a least safer setters/getters 

  wam_struct * wam;
  FILE *debugfile;

  bool debug;
  double moveToPos[7]; // buffer to store target joint angles 
  bool movingToPos;

  // buffers to store cartesian move information
  double moveToCartPos[3];
  double moveToCartOrient[3];
  bool movingToPosCart;

  std::string wamconf; // the wam config file

  int  startDone; // is the wam started ? 

  bool active[7];   // Should the joints be active or passive?
  //  double m_Jref[7]; // read/write joints buffer
  
  double m_Jtrq[7]; // read torques buffer

  vect_3 *RXRYRZ; // store wam orientation information

  WamNode();

  void init(std::string &);    //Take care of all the WAM startup stuff
  void cleanup(); //And shut it all down

  void initMoves();  // gentle stretch when waking the WAM up. 

  /** move to posture 
   *  @param pos : a double[7] array of joint angles 
   *  @param wait: wether to block until given posture is reached 
   */
  void goTo(const double * pos ,bool wait = true); 

  /** move to cartesian coord
   * @param pos : a double[3] array of x,y,z coordinates (in meters)
   * @param orient : a double[3] array of rx,ry,rz orientations (euler angles) 
   */
  void goToCart(const double * pos, const double * orient, bool wait = true);

  double * getJoints();
  double * getJointsCommand();
  double * getTorques();
  double * getTotalTorques();
  double * getGravityTorques();
  double * getMotorTorques();
  double * getMotorAngles();
  double * getCartesianOrientation(); // RX,RY,RZ
  double * getCartesianPosition(); // X,Y,Z
  double * getCartesianCommand();
  double * getCartesianTargets();
  double * getHomogeneousMatrix(); // Full cartesian homogeneous matrix.
  int getActiveSC(); // debug stuff
  int getZeroed(); // debug stuff
  Mode getCurrentMode(); // Get the current mode (cartesian or joints).
  Mode switchSpace(Mode); // Toggle between joint and cartesian spaces.
  void pos2HM(const double *, const double *, vect_3 *, matr_h *);
  void hm2Pos(double *, double *, vect_3 *, matr_h *);

  void goHome()
  {
    this->goTo(homePosition,false);
  }

  // stores current target buffer at 
  // provided address, if not directy
  // available, uses last read target. 
  // this method is intented to be used 
  // only by the realtime thread. 

  bool RTgetTargetJoints(double * joints); 
  bool RTupdateTargetJoints(const double * target);

  void setTargetJoints(const double * joints);

  bool RTgetTargetPosition(double *pos);
  bool RTupdateTargetPosition(const double *pos);

  void setTargetPosition(const double *pos);
  
  virtual ~WamNode(){};
};



// c style functions for threads and callbacks.. 
void rt_thread(void *thd_);
int WAMcallback(struct btwam_struct *m_wam);

