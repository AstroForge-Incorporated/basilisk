%module accelerometer
%{
   #include "accelerometer.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"
%include "std_vector.i"
%include "../_GeneralModuleFiles/af_swig_eigen.i"

%include "sys_model.h"
%include "accelerometer.h"

%include "msgPayloadDefC/AccelMsgPayload.h"
struct AccelMsg_C;

%include "architecture/msgPayloadDefC/SCMassPropsMsgPayload.h"
struct SCMassPropsMsg_C;
%include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"
struct THROutputMsg_C;

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
