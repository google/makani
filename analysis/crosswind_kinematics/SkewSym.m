function out = SkewSym(x)

% Skew symmetric cross product operator
% cross(x,y) = SkewSym(x)*y

out = [0    -x(3)   x(2);
       x(3)   0    -x(1);
      -x(2)  x(1)   0];
