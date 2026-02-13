%module controller
%{
   #include "controller.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"

%include "sys_model.h"
%include "controller.h"

%include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
struct AttGuidMsg_C;
%include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
struct CmdTorqueBodyMsg_C;
%include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
struct ArrayMotorTorqueMsg_C;

%pythoncode %{
_AttitudeController_swig_init = AttitudeController.__init__

def _AttitudeController_init(self, extra_jitter=False, config_path=None):
    if config_path is None:
        import importlib.resources
        config_path = str(
            importlib.resources.files("af_basilisk_ext")
            / "ExternalModules" / "bsk_acs" / "config.toml"
        )
    _AttitudeController_swig_init(self, config_path, extra_jitter)

AttitudeController.__init__ = _AttitudeController_init

import sys
protectAllClasses(sys.modules[__name__])
%}
