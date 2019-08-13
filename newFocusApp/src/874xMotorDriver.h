/*
FILENAME...   874xMotorDriver.h
USAGE...      Motor driver support for the NewFocus 874x series of controllers

Based on ACRMotorDriver.h by:
Mark Rivers
March 28, 2011

== Modifications ==
2015-12-01 - Wayne Lewis - Modify for NewFocus 874x controllers
2019-08-09 - Christopher Artates - Expanding closed-loop support for 8743-CL and updated poll method
*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define motorUpdateIntervalString   "MOTOR_UPDATE_INTERVAL"
#define motorDeadbandString         "MOTOR_DEADBAND"
#define motorFollowingErrorString   "MOTOR_FOLLOWING_ERROR"

class epicsShareClass nf874xAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  nf874xAxis(class nf874xController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setEncoderPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus doMoveToHome();
  asynStatus setHighLimit(double highLimit);
  asynStatus setLowLimit(double lowLimit);

  /* Methods not in base asynMotorAxis class */
  asynStatus setUpdateInterval(double interval);
  asynStatus setDeadband(int deadband);
  asynStatus setFollowingError(int threshold);

private:
  nf874xController *pC_;        /**< Pointer to the asynMotorController to which this axis belongs.
                                  *   Abbreviated because it is used very frequently */
  char   axisName_[10];         /**< Name of each axis, used in commands to nf874x controller */
  int    lastDirection_;        /**< Last direction of motion (0 == neg, 1 == pos) */
  double encoderPosition_;      /**< Cached copy of the encoder position */

  // Members specific for closed-loop support
  int    limitChecking_;        /**< Flag if hard travel limit checking is enabled */
  double highLimit_;            /**< Value of positive direction hardware limit set */
  double lowLimit_;             /**< Value of negative direction hardware limit set */
  double updateInterval_;       /**< Time in seconds of closed-loop update interval */
  double deadband_;             /**< Positioning range for avoiding limit cycling */
  double followingError_;       /**< Maximum allowed tracking error threshold */ 

friend class nf874xController;
};

class epicsShareClass nf874xController : public asynMotorController {
public:
  nf874xController(const char *portName, const char *nf874xPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int level);
  nf874xAxis* getAxis(asynUser *pasynUser);
  nf874xAxis* getAxis(int axisNo);
  
protected:
  char modelName_[10];        /**< Model name of controller, for default open/closed-loop control config */

  /** Index numbers for closed-loop specific parameters */
  int motorUpdateInterval_;
  int motorDeadband_;
  int motorFollowingError_;

  int hasClosedLoopSupport_;  /**< Flag indicating if controller supports closed-loop functionality */

friend class nf874xAxis;
};
