# Basilisk Attitude Controller

Simple PD attitude controller, to be used in simulation testing while we wait for the Orb Astro Simulation Library to be fully integrated. Controller gains can be configured in `config.toml`.

## Outputs
The Attitude Controller produces two outputs:
- Requested body torque message, to be used in simulations where RWs are not enabled and instead torques are applied directly to the body. Expressed in the body frame.
- Requested motor torque message, to be used in simulations where RWs are enabled torques are requested from the RW motors. This motor torque is the *negative* of the requested body torque. Expressed in the body frame.

## Usage
When testing Perception, the controller can be run with `extra_jitter`, which decreases the derivative gain (thus resulting in large (~1-2°) oscillations). `extra_jitter` is by default off.
