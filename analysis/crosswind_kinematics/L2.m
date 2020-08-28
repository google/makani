function out = L2(theta)

% y axis euler rotation

c = cos(theta);
s = sin(theta);

out = [c 0 -s;...
       0 1 0 ;...
       s 0 c];