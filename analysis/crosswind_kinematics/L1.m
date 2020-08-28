function out = L1(phi)

% x axis euler rotation

c = cos(phi);
s = sin(phi);

out = [1 0 0;...
       0 c s;...
       0 -s c];
