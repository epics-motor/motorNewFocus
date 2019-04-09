TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = newFocus
# newFocus.dbd will be created and installed
DBD += newFocus.dbd

# newFocus.dbd will be made up from these files:
newFocus_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
newFocus_DBD += asyn.dbd
newFocus_DBD += drvAsynSerialPort.dbd
newFocus_DBD += drvAsynIPPort.dbd
#endif
newFocus_DBD += motorSupport.dbd
newFocus_DBD += devNewFocus.dbd

# Add all the support libraries needed by this IOC
newFocus_LIBS += NewFocus
newFocus_LIBS += motor
#ifdef ASYN
newFocus_LIBS += asyn
#endif
#ifdef SNCSEQ
newFocus_LIBS += seq pv
#endif

# newFocus_registerRecordDeviceDriver.cpp derives from newFocus.dbd
newFocus_SRCS += newFocus_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
newFocus_SRCS_DEFAULT += newFocusMain.cpp
newFocus_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#newFocus_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
newFocus_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

