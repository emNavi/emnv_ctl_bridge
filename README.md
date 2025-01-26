# control_for_gym
control_for_gym is a PX4 middleware stack to publish NN-inferenced commands to expected levels of PX4 control loop. It is integrated in `AirGym-Real` for sending commands to PX4 autopilot during the Sim-to-Real.

Two functions are primarily implemented: constructing a finite state machine (FSM) to enable switching between trained policy control and classical PID control; and forwarding control commands to the PX4 Autopilot controller based on the selected control hierarchy.

## Control bridge

- Four types of controls are implemented:
  - VEL + YAW
  - POS + YAW
  - ATTI + THRUST
  - RATE + THRUST
 
- PID controls in FSM:
  - Takeoff
  - Hovering

Use channal 8 to switch classic control and nerual network inference control.