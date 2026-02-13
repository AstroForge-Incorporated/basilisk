%module dummyAttDet
%{
   #include "dummyAttDet.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"

%include "sys_model.h"
%include "dummyAttDet.h"

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
struct NavAttMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
