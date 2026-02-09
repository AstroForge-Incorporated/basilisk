# AF Basilisk Extension Modules

Basilisk extension modules, created by engineers at AstroForge.

## Directory Structure
- `ExternalModules`: Basilisk External Modules. Note: DO NOT change the name of this directory; it requires this name in order for C++ modules living in this folder to be built with Basilisk.
- `msgPayloadDefC`: Custom C Basilisk messages used by the Basilisk extension modules
- `msgPayloadDefCpp`: Custom C++ Basilisk messages used by the Basilisk extension modules

## Static analysis
To statically check Python code in the directory, you can run `mypy .` from within the folder. `mypy` settings are found in `.mypy.ini`
