# [WIP] Basilisk Actuator Control Library
Note: This library is currently under development. The architecture is fluid and subject to change.

## Overview
This library is a hardware-agnostic library intended to support basic commutation of actuators.
As of now, the following actuator types with associated commutation strategy are supported by this C++ library:
- BLDC/PMSM (Sensored & Sensorless Trap, Sensored FOC)
- Stepper Motor Commutation (Note: SW commutation of stepper motors should be avoided as the motor will commutate far slower)
- Brushed Motor (Torque and Open-Loop Speed)

## Architecture
TBC

## References
See the simulation hook-in here that employs Simulink. It is very scrappy at the moment and was used primarily for elementary modelling exposure and verification of actuator control strategies implemented in this library off-target. https://github.com/sahil-kale/basilisk-control-loop-sim