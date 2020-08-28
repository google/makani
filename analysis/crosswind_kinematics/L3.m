function out = L3(psi)

% z axis euler rotation

c = cos(psi);
s = sin(psi);

out = [c s 0;...
      -s c 0;...
       0 0 1];
