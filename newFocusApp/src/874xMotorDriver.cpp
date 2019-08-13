/*
FILENAME... 874xMotorDriver.cpp
USAGE...    Motor driver support for the NewFocus 874x series motor controller

Based on ACRMotorDriver.cpp by:
Mark Rivers
March 4, 2011

== Modifications ==
2015-12-01 - Wayne Lewis - Modify for NewFocus 874x series
2019-08-09 - Christopher Artates - Added closed-loop functionality
                                   Modified home, poll functions
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "874xMotorDriver.h"

#define NUM_nf874x_PARAMS 3

static const char *driverName = "nf874xMotorDriver";

/** Creates a new nf874xController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] nf874xPortName    The name of the drvAsynIPPPort that was created previously to connect to the nf874x controller 
  * \param[in] numAxes           The number of axes that this controller supports. Create one extra axis to allow for base 1 indexing of NewFocus controllers. 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
nf874xController::nf874xController(const char *portName, const char *nf874xPortName, int numAxes, 
                             double movingPollPeriod, double idlePollPeriod)
  //:  asynMotorController(portName, numAxes, NUM_nf874x_PARAMS, 
  :  asynMotorController(portName, numAxes+1, NUM_nf874x_PARAMS,
                         asynUInt32DigitalMask, 
                         asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  static const char *functionName = "nf874xController";

  /* Connect to nf874x controller */
  status = pasynOctetSyncIO->connect(nf874xPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect to nf874x controller\n",
      driverName, functionName);
  }
  epicsThreadSleep(0.5);
  
  // Retrieve controller model name (8742/8743-CL)
  // Set default control mode (open, closed) based on model
  sprintf(this->outString_, "VE?");
  this->writeReadController();
  strncpy(modelName_, this->inString_, 9);
  char *spc = strchr(modelName_, ' ');
  *spc = '\0';
  
  if(strcmp(modelName_, "8743-CL") == 0)
    hasClosedLoopSupport_ = true;
  else
    hasClosedLoopSupport_ = false;
  
  // New parameters specific to 8743-CL
  createParam(motorUpdateIntervalString,    asynParamFloat64,    &motorUpdateInterval_);
  createParam(motorDeadbandString,          asynParamInt32,      &motorDeadband_);
  createParam(motorFollowingErrorString,    asynParamInt32,      &motorFollowingError_); 

  // Create the axis objects
  // Axis 0 will remain unused. This allows consistent axis numbering with 
  // NewFocus convention.
  for (axis=1; axis<=numAxes; axis++) {
    new nf874xAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new nf874xController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] nf874xPortName    The name of the drvAsynIPPPort that was created previously to connect to the nf874x controller 
  * \param[in] numAxes           The number of axes that this controller supports.
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int nf874xCreateController(const char *portName, const char *nf874xPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  new nf874xController(portName, nf874xPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp    The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void nf874xController::report(FILE *fp, int level)
{
  fprintf(fp, "nf874x motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an nf874xMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
nf874xAxis* nf874xController::getAxis(asynUser *pasynUser)
{
  return static_cast<nf874xAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an nf874xMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
nf874xAxis* nf874xController::getAxis(int axisNo)
{
  return static_cast<nf874xAxis*>(asynMotorController::getAxis(axisNo));
}


/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * For all other functions it calls asynMotorController::writeInt32.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus nf874xController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  nf874xAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";
  
  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);

  if (function == motorDeadband_) {
    status = pAxis->setDeadband(value);
  }
  else if (function == motorFollowingError_) {
    status = pAxis->setFollowingError(value);
  }
  else {
    status = asynMotorController::writeInt32(pasynUser, value);
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%d, value=%d\n", 
        driverName, functionName, status, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%d, value=%d\n", 
        driverName, functionName, function, value);
  return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * For all other functions it calls asynMotorController::writeFloat64.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus nf874xController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  nf874xAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeFloat64";
   
  /* Set the parameter and readback in the parameter library. */
  status = setDoubleParam(pAxis->axisNo_, function, value);
  
  if (function == motorUpdateInterval_) {
    status = pAxis->setUpdateInterval(value);
  }
  else {
    status = asynMotorController::writeFloat64(pasynUser, value);
  }

  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%d, value=%f\n", 
        driverName, functionName, status, function, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%d, value=%f\n", 
        driverName, functionName, function, value);
  return status;
}

// These are the nf874xAxis methods

/** Creates a new nf874xAxis object.
  * \param[in] pC     Pointer to the nf874xController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_.
  * 
  * Initializes register numbers, etc.
  */
nf874xAxis::nf874xAxis(nf874xController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  sprintf(axisName_, "%d", axisNo);
  
  if(pC->hasClosedLoopSupport_) {
    // Retrieve limit switch statuses
    // If both positive and negative limits are triggered (feedback not present),
    //   disable travel limit checking (e.g. 8410 rotary picomotor)
    sprintf(pC_->outString_, "PH?");
    pC_->writeReadController();
    int motorType_ = atoi(pC_->inString_);
    limitChecking_ = (motorType_ & (0x3 << 3 * (axisNo - 1))) != (0x3 << 3 * (axisNo - 1));

    setIntegerParam(pC->motorStatusGainSupport_, 1); // Closed-loop positioning supported
    setIntegerParam(pC->motorStatusHasEncoder_,  1); // Encoder feedback assumed present
    callParamCallbacks();
    setClosedLoop(true);
  }
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void nf874xAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_ );
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

/** Move to position relative to home or current position.
  * \param[in] position     Absolute position to move to or relative distance to move. Units set with "SN".
  * \param[in] relative     Flag indicating relative move (1) or absolute move (0).
  * \param[in] minVelocity  The initial velocity. Units=steps/sec.
  * \param[in] maxVelocity  The maximum velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus nf874xAxis::move(double position, int relative, double minVelocity,
                            double maxVelocity, double acceleration)
{
  asynStatus status;
  double targetPosition_;
  
  sprintf(pC_->outString_, "%s AC %f", axisName_, acceleration);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s VA %f", axisName_, maxVelocity);
  status = pC_->writeController();

  if (relative) {
    sprintf(pC_->outString_, "%s PR %f", axisName_, position);
    targetPosition_ = encoderPosition_ + position;
  } else {
    sprintf(pC_->outString_, "%s PA %f", axisName_, position);
    targetPosition_ = position;
  }
  
  status = pC_->writeController();
  lastDirection_ = targetPosition_ > encoderPosition_;
  return status;
}

/** Begin indexing for home position in direction/
  * \param[in] minVelocity   The initial velocity. Units=steps/sec.
  * \param[in] maxVelocity   The maximum velocity. Units=steps/sec.
  * \param[in] acceleration  The acceleration value. Units=steps/sec/sec.
  * \param[in] forwards      Flag indicating move in the forward(1) or reverse direction(0).
  * If home switch is not supported (i.e. 8310 picomotor), home via limit indexing
  * If travel limits not enabled/supported (8410 rotary stage), use homing command */
asynStatus nf874xAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  
  sprintf(pC_->outString_, "%s AC %f", axisName_, acceleration);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s VA %f", axisName_, maxVelocity);
  status = pC_->writeController();

  if(limitChecking_)
    sprintf(pC_->outString_, "%s MT %s", axisName_, forwards ? "+" : "-");
  else
    sprintf(pC_->outString_, "%s MZ %s", axisName_, forwards ? "+" : "-");

  status = pC_->writeController();
  lastDirection_ = forwards;
  return status;
}

/* Move indefinitely in direction with specified velocity
  * \param[in] minVelocity  The initial velocity. Units=steps/sec.
  * \param[in] maxVelocity  The maximum velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus nf874xAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  double speed=maxVelocity;
  int forwards=1;

  if (speed < 0) {
    speed = -speed;
    forwards = 0;
  }
  sprintf(pC_->outString_, "%s AC %f", axisName_, acceleration);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s VA %f", axisName_, speed);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s MV %s", axisName_, forwards ? "+" : "-");
  status = pC_->writeController();
  lastDirection_ = forwards;
  return status;
}

/* Stop motor with deceleration 
 * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus nf874xAxis::stop(double acceleration)
{
  asynStatus status;

  sprintf(pC_->outString_, "%s ST", axisName_);
  status = pC_->writeController();
  return status;
}

/* Assign current motor position as specified steps displaced from home (0).
 * \param[in] position The new absolute motor position. Units=steps. */
asynStatus nf874xAxis::setPosition(double position)
{
  asynStatus status;

  sprintf(pC_->outString_, "%s DH %f", axisName_, position);
  status = pC_->writeController();
  return status;
}

/** Assign current encoder position as specified counts displaced from home (0).
  * \param[in] position The new absolute encoder position. Units=counts. */
asynStatus nf874xAxis::setEncoderPosition(double position)
{
  asynStatus status;

  sprintf(pC_->outString_, "%s DH %f", axisName_, position);
  status = pC_->writeController();
  return status;
}

/** Enabling Closed-loop Control
  * - Ensure use of "Count" units (using steps disables closed-loop control) via xxSN1
  * - Configure Hardware to enable/disable Travel Limit and Following Error
  *     Checking (to enable, set bit 0 and bit 1 to '0')
  *     For rotary positioners, disable travel limit checking
  *       (set bit 0 to '1', eg "%s ZH1")
  * - Enable Closed-loop positioning via xxMM1.
  * \param[in] closedLoop  true = closed loop, false = open loop.
  */
asynStatus nf874xAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status;

  if (closedLoop) {
    sprintf(pC_->outString_, "%sSN1;%sZH%i;%sMM%i",
            axisName_, axisName_, limitChecking_ ? 0 : 1, axisName_, closedLoop);
  }
  else {
    sprintf(pC_->outString_, "%sSN0;%sZH3;%sMM%i",
            axisName_, axisName_, axisName_, closedLoop);
  }
  status = pC_->writeController();
  setIntegerParam(pC_->motorStatusPowerOn_, closedLoop);
  callParamCallbacks();
  return status;
}

/** Move directly to home position without indexing, if already defined via
  *   setPosition or setEncoderPosition
  * Note: home position of absolute 0, not necessarily home switch position
  */
asynStatus nf874xAxis::doMoveToHome()
{
  asynStatus status;
  
  sprintf(pC_->outString_, "%s PA 0", axisName_);
  status = pC_->writeController();
  lastDirection_ = encoderPosition_ < 0;
  return status;
}

/** Set the high limit position of the motor.
  * \param[in] highLimit The new high limit position that should be set in the hardware. 
  *   Units set with "SN". ZH bit 0 must be set to 0 to follow limit checking. */
asynStatus nf874xAxis::setHighLimit(double highLimit) {
  asynStatus status;

  sprintf(pC_->outString_, "%s SR %f", axisName_, highLimit);
  status = pC_->writeController();
  highLimit_ = highLimit;
  return status;
}

/** Set the low limit position of the motor.
  * \param[in] lowLimit The new low limit position that should be set in the hardware.
  *   Units set with "SN". ZH bit 0 must be set to 0 to follow limit checking. */
asynStatus nf874xAxis::setLowLimit(double lowLimit) {
  asynStatus status;

  sprintf(pC_->outString_, "%s SL %f", axisName_, lowLimit);
  status = pC_->writeController();
  lowLimit_ = lowLimit;
  return status;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status. It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus nf874xAxis::poll(bool *moving)
{ 
  int done;
  asynStatus comStatus;

  // Read the current encoder position
  sprintf(pC_->outString_, "%s TP?", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  encoderPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorEncoderPosition_,encoderPosition_);

  // Read the current theoretical position
  setDoubleParam(pC_->motorPosition_, encoderPosition_);

  // Read current/last direction of motion
  setIntegerParam(pC_->motorStatusDirection_, lastDirection_);

  // Read the current moving statuses
  sprintf(pC_->outString_, "%s MD?", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  done = atoi(pC_->inString_);
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false : true;
  setIntegerParam(pC_->motorStatusMoving_, *moving);

  // Read soft home status (motor at absolute '0'?)
  if(encoderPosition_ == 0.0)
    setIntegerParam(pC_->motorStatusHome_, 1);
  else
    setIntegerParam(pC_->motorStatusHome_, 0);
  
  // Read closed-loop and limit switch statuses if supported
  if(pC_->hasClosedLoopSupport_) {
    int clen;
    sprintf(pC_->outString_, "%s MM?", axisName_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    clen = atoi(pC_->inString_);
    setIntegerParam(pC_->motorStatusPowerOn_, clen);
    
    sprintf(pC_->outString_, "%s SR?", axisName_);
    comStatus = pC_->writeReadController();
    if(comStatus) goto skip;
    highLimit_ = atof(pC_->inString_);
    setDoubleParam(pC_->motorHighLimit_, highLimit_);

    sprintf(pC_->outString_, "%s SL?", axisName_);
    comStatus = pC_->writeReadController();
    if(comStatus) goto skip;
    lowLimit_ = atof(pC_->inString_);
    setDoubleParam(pC_->motorLowLimit_, lowLimit_);

    // Hard limit switch status checking
    // bit 0 - state of positive limit switch
    // bit 1 - state of negative limit switch
    // bit 2 - state of home limit switch
    // Repeated for additional axes
    // If feedback cable/limit sensor is not connected, switch presents as triggered.
    int lmst;
    sprintf(pC_->outString_, "PH?");
    comStatus = pC_->writeReadController();
    if(comStatus) goto skip;
    lmst = atoi(pC_->inString_);
    bool bit_;
    for(int i = 0; i < 3; i++) {
      bit_ = lmst & ( 0x1 << (i + 3 * (axisNo_ - 1)));
      switch(i) {
        // Positive and negative limit switches not present on 8410, statuses disabled
        case 0:  setIntegerParam(pC_->motorStatusHighLimit_, limitChecking_ ? bit_ : 0);
                 break;
        case 1:  setIntegerParam(pC_->motorStatusLowLimit_,  limitChecking_ ? bit_ : 0);
                 break;
        // Note: Home switch not present on 8310, will always appear as '1'.
        default: setIntegerParam(pC_->motorStatusAtHome_, bit_);
      }
    }
  }

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1 : 0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Set the updating interval for closed-loop correction.
  * \param[in] interval Time in seconds between static position regulation
  */
asynStatus nf874xAxis::setUpdateInterval(double interval)
{
  asynStatus status;
  
  sprintf(pC_->outString_, "%s CL %f", axisName_, interval);
  status = pC_->writeController();
  updateInterval_ = interval;
  
  return status;
}

/** Set the deadband for closed-loop correction.
  * \param[in] deadband Max encoder counts allowed between target and final actual motor positions
  */
asynStatus nf874xAxis::setDeadband(int deadband)
{
  asynStatus status;
  
  sprintf(pC_->outString_, "%s DB %d", axisName_, deadband);
  status = pC_->writeController();
  deadband_ = deadband;
  
  return status;
}

/** Set the following error threshold for closed-loop correction.
  * \param[in] threshold Max encoder counts allowed between actual and theoretical current motor positions
  */
asynStatus nf874xAxis::setFollowingError(int threshold)
{
  asynStatus status;
  
  sprintf(pC_->outString_, "%s FE %d", axisName_, threshold);
  status = pC_->writeController();
  followingError_ = threshold;

  return status;
}

/** Code for iocsh registration */
static const iocshArg nf874xCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg nf874xCreateControllerArg1 = {"nf874x port name", iocshArgString};
static const iocshArg nf874xCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg nf874xCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg nf874xCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const nf874xCreateControllerArgs[] = {&nf874xCreateControllerArg0,
                                                           &nf874xCreateControllerArg1,
                                                           &nf874xCreateControllerArg2,
                                                           &nf874xCreateControllerArg3,
                                                           &nf874xCreateControllerArg4};
static const iocshFuncDef nf874xCreateControllerDef = {"nf874xCreateController", 5, nf874xCreateControllerArgs};
static void nf874xCreateControllerCallFunc(const iocshArgBuf *args)
{
  nf874xCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void nf874xMotorRegister(void)
{
  iocshRegister(&nf874xCreateControllerDef, nf874xCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(nf874xMotorRegister);
}

