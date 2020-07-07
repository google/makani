// No need to specify a module name.
%include "stdint.i"
%include "portion_b.i"
%{
// Put headers and other declarations here.
#include "lib/bazel/swig_test/fruit.h"
#include "avionics/common/cvt_avionics_messages.h"
extern double PortionA(uint16_t, uint16_t);
%}

%include "avionics/common/cvt_avionics_messages.h"
%import "avionics/network/aio_node.h"

%include "cpointer.i"
%pointer_functions(double, double_pointer);
%apply double *OUTPUT {double *total}
%include "lib/bazel/swig_test/fruit.h"

extern double PortionA(uint16_t, uint16_t);
