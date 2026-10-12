The process for creating this module was as follows:

- Started with a copy of the prescribedRotation1DOF module
- Removed all logic and variables related to smoothed, variable acceleration
- Modified logic to limit the maximum velocity of the rotation to user specified value
- Derived and implemented modified logic to be able to account for:
  - Acceleration only up to a max velocity, then coasting occurs. If the max velocity is not hit, the maneuver is bang-bang, if the max velocity is hit, the maneuver is bang-coast-bang
  - non-zero initial rates in both bang-bang and bang-coast-bang modes

## Nonzero initial rate

A target change beyond `newTargetAngularThreshold` can arrive mid-rotation, so the first segment may start at
`thetaDotInit != 0`. Its planned displacement is then `0.5 * thetaDDot * dt_b1^2 + thetaDotInit * dt_b1`, which is how
far the first segment actually turns when it is integrated. Without the `thetaDotInit * dt_b1` term, the next segment
would start that far short of the body's actual angle, and the body would jump back by that amount when the first
segment ends.
