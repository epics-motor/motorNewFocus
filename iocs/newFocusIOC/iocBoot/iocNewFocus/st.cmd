#!../../bin/linux-x86_64/newFocus

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/newFocus.dbd"
newFocus_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=newFocus:")

##
< PMNC87xx.cmd
##
< newfocus8742.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("newFocus:")

# Boot complete
