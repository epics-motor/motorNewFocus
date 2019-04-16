# motorNewFocus
EPICS motor drivers for the following [New Focus](https://www.newport.com/b/new-focus) controllers: 8750, 8752, & 874x

motorNewFocus is a submodule of [motor](https://github.com/epics-modules/motor).  When motorNewFocus is built in the ``motor/modules`` directory, no manual configuration is needed.

motorNewFocus can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorNewFocus contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
