# Accelerometer Module

Custom Basilisk module representing an accelerometer, including accelerometer noises.

## Modeled Noise Sources

1. Scale factor drift
2. Bias drift
3. White noise
4. Quantization
5. Limits
6. Deadbands

## Directory Structure
- `_UnitTest`: Module unit tests, runnable via Pytest
- `accel_config.py`: Configuration file for setting up multiple accelerometer models. E.g., to generate a model of the [ADXL-356-EP](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl356-ep.pdf) that we're planning on using in MS-2, use the `ADXL356Factory`. To generate a perfect accelerometer with no noise, use the `PerfectAccelFactory`
- `accelerometer.cpp`: Accelerometer implementation
- `accelerometer.h`: Accelerometer header file
- `accelerometer.i`: File used as part of Basilisk's SWIG logic to allow the module to be called and configured from Python

## Static analysis
To statically check Python code in the directory, you can run `mypy .` from within the folder. `mypy` settings are found in `.mypy.ini`
