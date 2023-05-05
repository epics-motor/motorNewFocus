# motorNewFocus Releases

## __R1-2 (2023-05-05)__
R1-2 is a release based on the master branch.

### Changes since R1-1-1

#### New features
* None

#### Modifications to existing features
* Pull request [#8](https://github.com/epics-motor/motorNewFocus/pull/8): Simplified the newfocus8742 example. Made it easier to copy to a different IOC.

#### Bug fixes
* None

#### Continuous integration
* Added ci-scripts (v3.0.1)
* Switched from Travis CI to Github Actions

## __R1-1-1 (2020-05-12)__
R1-1-1 is a release based on the master branch.  

### Changes since R1-1

#### New features
* None

#### Modifications to existing features
* None

#### Bug fixes
* Commit [0000255](https://github.com/epics-motor/motorNewFocus/commit/0000255ab0a98ba2f57c4b485cddc661500815fc): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``
* Pull request [#2](https://github.com/epics-motor/motorNewFocus/pull/2): Eliminated compiler warnings
* Pull request [#3](https://github.com/epics-motor/motorNewFocus/pull/3): [Kaz Gofron](https://github.com/kgofron) added documenation for supported 874x firmware versions

## __R1-1 (2019-08-13)__
R1-1 is a release based on the master branch.  

### Changes since R1-0

#### Modifications to existing features
* Pull request [#1](https://github.com/epics-motor/motorNewFocus/pull/1) Multiple improvements to 874xMotorDriver from [Chris Arta](https://github.com/ArtaChris)

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorNewFocus is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorNewFocus can be built outside of the motor directory
* motorNewFocus has a dedicated example IOC that can be built outside of motorNewFocus

#### Modifications to existing features
* None

#### Bug fixes
* None
