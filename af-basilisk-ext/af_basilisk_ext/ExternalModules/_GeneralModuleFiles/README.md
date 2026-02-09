# General Module Files

General support files used across modules.

## Directory Structure
- `af_swig_eigen.i`: Custom AstroForge support file for allowing the user to access Eigen functions from Python via SWIG
- `eigenSupport.h`: Convenience header for converting C arrays to Eigen arrays, and printing arrays for debugging
- `random.h`: Custom header that allows user to build an Eigen matrix of random values, with similar symantics as numpy's `random.normal()` and `random.uniform()` functions
