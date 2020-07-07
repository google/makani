// Any module name specified here will be overwritten by the rule in BUILD.
%module useless
%include "stdint.i"
%include "percent.i"

%{
// Put headers and other declarations here.
#include "lib/bazel/swig_test/fruit.h"
#include "avionics/common/cvt_avionics_messages.h"
extern double MissRate(uint16_t, uint16_t);
%}

%include "avionics/common/cvt_avionics_messages.h"
%import "avionics/network/aio_node.h"

%include "cpointer.i"
%pointer_functions(double, double_pointer);
%apply double *OUTPUT {double *total}
%include "lib/bazel/swig_test/fruit.h"

extern double MissRate(uint16_t, uint16_t);
