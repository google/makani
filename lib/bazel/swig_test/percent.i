// No need to specify a module name.
%include "stdint.i"

%{
extern double Percent(uint16_t, uint16_t);
%}

extern double Percent(uint16_t, uint16_t);
