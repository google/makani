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

function [d, ib, test] = findnearest(a, b)
% Find the nearest elements in array 'b' to the elements in array 'a'.
% Arguments:
% 	a: target value
% 	b: vector searched
% Returns:
% 	d: absolute difference between "a" and nearest element in "b"
% 	ib: index of the nearest element in "b"
% 	test: boolean: true if a lies between boundaries of b
%
% The following assumes that 'a' and 'b' are row vectors and that all the
% elements of 'a' are finite. It returns the two row vectors 'd' and 'ib'
% of the same length as 'a'. Each element of 'd' is the absolute difference
% between the corresponding element of 'a' and the nearest element of 'b'.
% Each element of 'ib' is the index with respect to the 'b' vector of that
% corresponding nearest 'b' element.

m = size(a, 1);
n = size(b, 2);

[~, p] = sort([a, b]);

q = 1:m+n;
q(p) = q;

t = cumsum(p>m);

r = 1:n;
r(t(q(m+1:m+n))) = r;

s = t(q(1:m));

id = r(max(s,1));
iu = r(min(s+1, n));

[d, it] = min([abs(a-b(id)); abs(b(iu)-a)]);

ib = id+(it-1).*(iu-id);

if ( a>=min(b) ) && ( a<=max(b) )
    test = true;
else
    test = false;
end

end
