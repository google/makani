%{
  #include "avionics/common/avionics_messages.h"
  #include "avionics/common/cvt.h"
  #include "avionics/network/message_type.h"
  #include "avionics/linux/aio.h"
  #include "avionics/linux/clock.h"
  #include "avionics/linux/swig/aio_util.h"
  #include "gs/monitor2/high_frequency_filters/common.h"

  #define SWIG_FILE_WITH_INIT
%}

%include "stdint.i"
%include "typemaps.i"
%include "third_party/swig_lib/numpy.i"
%init %{
   import_array();
%}

%import "avionics/network/aio_node.h"
%import "avionics/network/message_type.h"

%include "avionics/common/avionics_messages.h"
%include "avionics/linux/clock.h"

// Tell SWIG to interpret such pointer arguments as outputs (for CvtGet).
// This produces a Python CvtGet function that can be used as:
// buffer, sequence, timestamp = CvtGet(source, message_type)
// When there is new message, the Python wrapper returns (buffer, sequence,
// timestamp). If there is no new message, `buffer` becomes NULL, and it only
// returns (sequence, timestamp).
%apply uint16_t *OUTPUT {uint16_t *sequence}
%apply int64_t *OUTPUT {int64_t *timestamp}

// For SWIG to interpret the argument to GetFilteredData(unsigned long long* length)
// as an output.
%apply uint32_t *OUTPUT {uint32_t *length}
%include "gs/monitor2/high_frequency_filters/common.h"

// Tell SWIG to wrap such arguments as a string in Python (for CvtPut).
%typemap(in) (const void *buf, int32_t len) {
  if (!PyString_Check($input)) {
      PyErr_SetString(PyExc_ValueError, "Expecting a string");
      return NULL;
  }
  $1 = (void *) PyString_AsString($input);
  Py_ssize_t length = PyString_Size($input);
  if (length < 0 || length > INT32_MAX) {
      PyErr_SetString(PyExc_ValueError, "Integer out of range.");
      return NULL;
  }
  $2 = (int32_t)length;
}
%include "avionics/common/cvt.h"

// Tell SWIG to interpret such arguments as an input integer array.
%typemap(in) (const MessageType *subscribe_types, int32_t num_subscribe_types) {
  if (!PyList_Check($input)) {
    PyErr_SetString(PyExc_ValueError, "Expecting a list.");
    return NULL;
  }
  Py_ssize_t length = PyList_Size($input);
  if (length < 0 || length > INT32_MAX) {
      PyErr_SetString(PyExc_ValueError, "Integer out of range.");
      return NULL;
  }
  $2 = (int32_t)length;

  MessageType _message_types[kNumMessageTypes];
  $1 = _message_types;
  for (int32_t i = 0; i < $2; i++) {
    PyObject *s = PyList_GetItem($input, i);
    if (!PyInt_Check(s)) {
        PyErr_SetString(PyExc_ValueError, "List items must be integers.");
        return NULL;
    }

    long l = PyInt_AsLong(s);
    if (l < 0 || l >= kNumMessageTypes) {
        PyErr_SetString(PyExc_ValueError, "List item out of range.");
        return NULL;
    }
    $1[i] = l;
  }

  // SWIG enforces a type conversion when calling the C function. E.g,
  // AioSetup(arg1, arg2, (enum MessageType const *)arg3, arg4);
  // This however leads to a compilation error:
  //  "error: passing argument 3 of ‘AioSetup’ from incompatible pointer type."
  // The current hack is to define "enum" as empty, and undef it when freeing
  // the array below.
  %#define enum
}

%typemap(freearg) (const MessageType *subscribe_types,
                   int32_t num_subscribe_types) {
  %#undef enum
}

%include "avionics/linux/aio.h"

// Tell SWIG to interpret the arguments as a 2D array to be modified in-place.
// This is for converting GetAioUpdates to Python.
%apply (unsigned long long *INPLACE_ARRAY2, int DIM1, int DIM2) {
    (unsigned long long *aio_updates, int num_aio_nodes, int num_message_types)
}

%include "avionics/linux/swig/aio_util.h"
