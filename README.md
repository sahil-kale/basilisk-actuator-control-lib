# [WIP] Basilisk Actuator Control Library
Note: This library is currently under development. The architecture is fluid and subject to change.

## Overview
This library is a hardware-agnostic library intended to support basic commutation of actuators.
As of now, the following actuator types with associated commutation strategy are supported by this C++ library:
- BLDC/PMSM (Sensored & Sensorless Trap, Sensored FOC)

The following were, at one point, WIP, but are deprioritized in favour of a focus on BLDC software development.
- Stepper Motor Commutation (Note: SW commutation of stepper motors should be avoided as the motor will commutate far slower)
- Brushed Motor (Torque and Open-Loop Speed)

## Library Architecture
![High Level Architecture](https://github.com/sahil-kale/basilisk-actuator-control-lib/assets/32375512/a86fd9e6-1148-4635-a11f-be8f48c2e8cb)

The components shown in blue are implemented in this library; the user must implement the other modules as per their application. This affords flexibility when implementing the library as the user can adapt the various modules as required by the specific application.

## Development
The following commands are intended to be run on an Ubuntu 22.04 machine (and what is done right now in CI/CD). https://github.com/sahil-kale/basilisk-actuator-control-lib/issues/31 will track an eventual build system containerization. Follow the steps below for development:
1. Clone this repository
2. Run `bash scripts/setup.sh` - this will install packages on the system that are required for development.
3. Run `bash scripts/test.sh` to build the test cases.
4. Run `bash scripts/format.sh` to format the source code.
5. Run `bash scripts/linter.sh` to run the linter.

### API Documentation
API documentation is available [here](https://sahil-kale.github.io/basilisk-actuator-control-lib/)

### PR Submissions
Submit PR's against `main`. PR's that modify the source code's functionality without a unit test will not be accepted unless determined that the functionality does not require one. CI will check test, lint, and format upon check-in.

General styling:
- Doxygen-style comments on enums and function declarations (including internal functions)
- Write expressive code (use enums, make the codebase human-readable)

## References
See the simulation hook-in here that employs Simulink. It is very scrappy at the moment and was used primarily for elementary modelling exposure and verification of actuator control strategies implemented in this library off-target. https://github.com/sahil-kale/basilisk-control-loop-sim
