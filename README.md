# motorNewFocus
EPICS motor drivers for the following [New Focus](https://www.newport.com/b/new-focus) controllers: 8750, 8752, & 874x

[![Build Status](https://travis-ci.org/epics-motor/motorNewFocus.png)](https://travis-ci.org/epics-motor/motorNewFocus)

motorNewFocus is a submodule of [motor](https://github.com/epics-modules/motor).  When motorNewFocus is built in the ``motor/modules`` directory, no manual configuration is needed.

motorNewFocus can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorNewFocus contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.

The 874x controller requires firmware v2.3 for units produced prior to early 2017. For units after early 2017, the driver works with firmware v4.06. The v2.2 is missing commands (SR/SL) used by the EPICS driver.
In early 2017, firmware v4.06 was released. This was due to a revision in MCU/processors in which the whole MCU/processor changed. Controllers with the newer processing unit cannot be upgraded from 2.x to 4.x. However, older units with  v2.2 should be able to be upgraded to v2.3. 
