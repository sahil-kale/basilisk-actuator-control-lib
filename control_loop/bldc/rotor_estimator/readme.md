## Rotor Estimator
The BLDC control loops require a way of knowing the rotor position in order to properly commutate the motor. This is done via an implementation of a `ElectricalRotorPosEstimator`, which can provide the control loops with the electrical angle to perform the relevant commutation on.

Documentation surrounding the electrical rotor position estimator can be found here: [Electrical Rotor Position Documentation](https://sahil-kale.github.io/basilisk-actuator-control-lib/classbldc__rotor__estimator_1_1ElectricalRotorPosEstimator.html).

### Sensorless FOC
A rotor flux observer is currently implemented for sensorless FOC. Further information about the implementation can be found in `sensorless_foc.md` 
