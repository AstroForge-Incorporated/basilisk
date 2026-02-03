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
import sys
protectAllClasses(sys.modules[__name__])
%}
