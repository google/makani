% Copyright 2020 Makani Technologies LLC
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

% Compute the sub-optimal quaternion rotation between two sets of vectors,
% where the first set of vectors is known perfectly.
function [q] = q_subopt(r1, b1, r2, b2)

r1 = normalize(r1);
b1 = normalize(b1);
r2 = normalize(r2);
b2 = normalize(b2);

% Handle singularity conditions.
d(1) = 1 + ( b1(1)*r1(1) - b1(2)*r1(2) - b1(3)*r1(3));
d(2) = 1 + (-b1(1)*r1(1) + b1(2)*r1(2) - b1(3)*r1(3));
d(3) = 1 + (-b1(1)*r1(1) - b1(2)*r1(2) + b1(3)*r1(3));
d(4) = 1 + ( b1(1)*r1(1) + b1(2)*r1(2) + b1(3)*r1(3));
if d(1) >= d(2) && d(1) >= d(3) && d(1) >= d(4)
  dd = 1;
  r1(2) = -r1(2);
  r1(3) = -r1(3);
  r2(2) = -r2(2);
  r2(3) = -r2(3);
elseif d(2) >= d(1) && d(2) >= d(3) && d(2) >= d(4)
  dd = 2;
  r1(1) = -r1(1);
  r1(3) = -r1(3);
  r2(1) = -r2(1);
  r2(3) = -r2(3);
elseif d(3) >= d(1) && d(3) >= d(2) && d(3) >= d(4)
  dd = 3;
  r1(1) = -r1(1);
  r1(2) = -r1(2);
  r2(1) = -r2(1);
  r2(2) = -r2(2);
else
  dd = 4;
end

b3 = cross(b1, b2);
r3 = cross(r1, r2);
c1 = cross(b1, r1);
p1 = b1 + r1;

mu = d(dd)*dot(b3,r3) - dot(b1,r3)*dot(r1,b3);
nu = dot(p1, cross(b3,r3));
rho = sqrt(mu*mu + nu*nu);

if mu >= 0,
  coeff = rho + mu;
  if coeff == 0
    q = q_null();
    dd = 4;
  else
    s = 1/(2*sqrt(rho*coeff*d(dd)));
    q.s = [coeff*d(dd)]*s;
    q.v = [coeff*c1 + nu*p1]*s;
  end
else
  coeff = rho - mu;
  if coeff == 0,
    q = q_null();
    dd = 4;
  else
    s = 1/(2*sqrt(rho*coeff*d(dd)));
    q.s = [nu*d(dd)]*s;
    q.v = [nu*c1 + coeff*p1]*s;
  end
end

% Handle singularity condition.
if dd ~= 4
  e=[0; 0; 0];
  e(dd) = 1;
  s = q.s;
  q.s = -dot(q.v,e);
  q.v = s*e - cross(q.v,e);
end


function v = normalize(v)
v = v / norm(v);
