The process for creating this fine sun sensor module was as follows:

- it begun as a copy of the Basilisk coarseSunSensor module
- we renamed all the coarse parameters to fine
- Superflous functions which were not required or didn't make sense were deleted (such as saturation functions)
- The noise parameters were modified to output 3 independent noise values instead of a singular value
- added the logic for the FSS (checked if sun vector outside of FOV, if yes, set vector to 0)
- added noise to the truth sun vector to get sensed vector
- modified the output messages for what we are interested in

Note that I left some of the logic and calculations from the coarseSunSensor module even if we do not use it right now as it could be helpful in the future if we intent to expand the simulation or what have you.
