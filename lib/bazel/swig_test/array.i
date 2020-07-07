%module array
%{
  #define SWIG_FILE_WITH_INIT
  #include "lib/bazel/swig_test/array.h"
%}

%include "third_party/swig_lib/numpy.i"
%init %{
  import_array();
%}

%include "stdint.i"

// We use numpy.i to handle arrays for SWIG. For more details, see
// http://docs.scipy.org/doc/numpy/reference/swig.interface-file.html
%apply (int *IN_ARRAY1, int32_t DIM1) {
  (const int *fruits, int32_t count)
}

// Tell SWIG to wrap the following pattern in a way where the Python argument
// is converted into an array of void types and a variable for array length.
%typemap(in) (const void *buffer, int32_t len) {
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
%include "lib/bazel/swig_test/array.h"
