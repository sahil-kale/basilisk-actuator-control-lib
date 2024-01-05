# Brushless Control Loop
## Overview
The brushless control loops are intended to enable FOC and trapezoidal control of Permenant-Magnet Synchronous and Brushless DC motors (PMSM and BLDC). A note that while the terminology of 'BLDC' is adopted throughout the codebase, the field oriented control technique can generally be employed on both PMSM and BLDC motors by assuming the back-emf waveform of the latter to be generally sinusoidal. 

Two control loops are defined:
`BrushlessFOCControlLoop`, which enables field oriented control of a BLDC motor.
`Brushless6StepControlLoop`, which enables trapezoidal control of a BLDC motor.

The specific API for both of these control loops can be found [here](https://sahil-kale.github.io/basilisk-actuator-control-lib/annotated.html).

Background info: TBC 

## 6 Step Control Loop

## FOC Control Loop
The FOC control loop is capable of controlling a 3-phase motor by applying a desired current vector induced by the stator. A typical FOC control loop is shown below and generally follows what is implemented by this library:
![Typical Sensored FOC](https://github.com/sahil-kale/basilisk-actuator-control-lib/blob/main/assets/typicalfoc.png)

Note that this control loop is capable of using multiple rotor position estimators, enabling sensorless fallback operation if desired (in addition to fully sensorless capability). See the API for further information, alongside the unit tests.

### State Machine
A visual diagram of the state transition and associated conditions for the FOC control loop is shown below:

![State Machine](https://github.com/sahil-kale/basilisk-actuator-control-lib/blob/main/assets/BrushlessFOCStateMachine.png)

### Control Flow
A high-level control flow diagram of the top-level control loop is shown below, alongside the control flow for the FOC current controller flow diagram.
![High level control flow diagra,](https://github.com/sahil-kale/basilisk-actuator-control-lib/blob/main/assets/BrushlessFOCHighLevelControlFlow.png)
![FOC Current Control Flow](https://github.com/sahil-kale/basilisk-actuator-control-lib/blob/main/assets/BrushlessFOC_FOCControlFlow_.png)

## Rotor Estimator
### Sensorless FOC
A rotor flux observer is currently implemented for sensorless FOC. Further information about the implementation can be found in `docs/sensorless_foc.md` 

### Phase Estimation Control Law
TBC and linked.