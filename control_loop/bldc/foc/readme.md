## Field Oriented Control Loop
The FOC control loop is capable of controlling a 3-phase motor by applying a desired current vector induced by the stator. A typical FOC control loop is shown below and generally follows what is implemented by this library:
![Typical Sensored FOC](https://github.com/sahil-kale/basilisk-actuator-control-lib/blob/main/assets/typicalfoc.png)

Note that this control loop is capable of using multiple rotor position estimators, enabling sensorless fallback operation if desired (in addition to fully sensorless capability). See the API for further information, alongside the unit tests.

### State Machine
A visual diagram of the state transition and associated conditions for the FOC control loop is shown below:

![State Machine](https://github.com/sahil-kale/basilisk-actuator-control-lib/blob/main/assets/BrushlessFOCStateMachine.png)

### Control Flow
A high-level control flow diagram of the top-level control loop is shown below, alongside the control flow for the FOC current controller flow diagram.
![High level control flow diagram](https://github.com/sahil-kale/basilisk-actuator-control-lib/blob/main/assets/BrushlessFOCHighLevelControlFlow.png)
![FOC Current Control Flow](https://github.com/sahil-kale/basilisk-actuator-control-lib/blob/main/assets/BrushlessFOC_FOCControlFlow_.png)

### Phase Estimation Control Law
TBC and linked.