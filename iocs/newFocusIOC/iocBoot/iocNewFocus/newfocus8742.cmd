
epicsEnvSet("PORT",   "NF1")
epicsEnvSet("IP_ADDR", "10.28.2.211")

# Setup IP port for 8742 
drvAsynIPPortConfigure("$(PORT)_ETH", "$(IP_ADDR):23")
asynOctetSetInputEos("$(PORT)_ETH",0,"\r\n")
asynOctetSetOutputEos("$(PORT)_ETH",0,"\r")

#db asyn debug traces
asynSetTraceMask("$(PORT)_ETH",-1,0x1)
asynSetTraceIOMask("$(PORT)_ETH",-1,0x1)

# New Focus Picomotor Network Controller (model 87xx) configuration parameters:  
#     (1) Controller asyn port name (string)
#     (2) IP asyn port name (string)
#     (3) Number of axes
#     (4) Moving poll period (ms)
#     (5) Idle poll period (ms)
nf874xCreateController("$(PORT)", "$(PORT)_ETH", 3, 200, 1000)

## Load record instances
dbLoadTemplate("newfocus8742.substitutions", "P=$(PREFIX),PORT=$(PORT)")

dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(PREFIX),R=asyn_1,PORT=$(PORT),ADDR=,OMAX=80,IMAX=80")
