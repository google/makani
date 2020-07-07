// No need to specify a module name.
%include "stdint.i"
%{
extern double PortionB(uint16_t, uint16_t);
%}

extern double PortionB(uint16_t, uint16_t);
