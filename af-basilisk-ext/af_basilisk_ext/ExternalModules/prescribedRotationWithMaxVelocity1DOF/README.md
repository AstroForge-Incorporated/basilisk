The process for creating this module was as follows:

- Started with a copy of the prescribedRotation1DOF module
- Removed all logic and variables related to smoothed, variable acceleration
- Modified logic to limit the maximum velocity of the rotation to user specified value
- Derived and implemented modified logic to be able to account for:
  - Acceleration only up to a max velocity, then coasting occurs. If the max velocity is not hit, the maneuver is bang-bang, if the max velocity is hit, the maneuver is bang-coast-bang
  - non-zero initial rates in both bang-bang and bang-coast-bang modes
