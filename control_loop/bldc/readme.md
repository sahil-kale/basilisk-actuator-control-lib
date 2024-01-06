# 3 Phase Control Loops
## Overview
The brushless control loops are intended to enable FOC and trapezoidal control of Permenant-Magnet Synchronous and Brushless DC motors (PMSM and BLDC). A note that while the terminology of 'BLDC' is adopted throughout the codebase, the field oriented control technique can generally be employed on both PMSM and BLDC motors by assuming the back-emf waveform of the latter to be generally sinusoidal. 

Two control loops are defined:
`BrushlessFOCControlLoop`, which enables field oriented control of a BLDC motor.
`Brushless6StepControlLoop`, which enables trapezoidal control of a BLDC motor.

The specific API for both of these control loops can be found [here](https://sahil-kale.github.io/basilisk-actuator-control-lib/annotated.html), with further information contained in the `readme.md` inside the respective subfolders.