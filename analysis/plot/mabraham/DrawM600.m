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

function H = DrawM600(x,y,z,dcm,rawm600,varargin)

rawm600.wing = rawm600.scale*(dcm'*rawm600.wing')';
rawm600.vtail = rawm600.scale*(dcm'*rawm600.vtail')';
rawm600.htail = rawm600.scale*(dcm'*rawm600.htail')';
rawm600.pylon1 = rawm600.scale*(dcm'*rawm600.pylon1')';
rawm600.pylon2 = rawm600.scale*(dcm'*rawm600.pylon2')';
rawm600.pylon3 = rawm600.scale*(dcm'*rawm600.pylon3')';
rawm600.pylon4 = rawm600.scale*(dcm'*rawm600.pylon4')';
rawm600.fuse = rawm600.scale*(dcm'*rawm600.fuse')';
rawm600.rotor = rawm600.scale*(dcm'*rawm600.rotor')';

hold on

if isempty(varargin)
    H.wing = fill3(y + rawm600.wing(:,2),x + rawm600.wing(:,1),-z - rawm600.wing(:,3),'b');
    H.p1 = fill3(y + rawm600.pylon1(:,2),x + rawm600.pylon1(:,1),-z - rawm600.pylon1(:,3),'g');
    H.p2 = fill3(y + rawm600.pylon2(:,2),x + rawm600.pylon2(:,1),-z - rawm600.pylon2(:,3),'g');
    H.p3 = fill3(y + rawm600.pylon3(:,2),x + rawm600.pylon3(:,1),-z - rawm600.pylon3(:,3),'g');
    H.p4 = fill3(y + rawm600.pylon4(:,2),x + rawm600.pylon4(:,1),-z - rawm600.pylon4(:,3),'g');
    H.htail = fill3(y + rawm600.htail(:,2),x + rawm600.htail(:,1),-z - rawm600.htail(:,3),'r');
    H.vtail = fill3(y + rawm600.vtail(:,2),x + rawm600.vtail(:,1),-z - rawm600.vtail(:,3),'c');
    H.f1 = fill3(y + rawm600.fuse([1 2 6 5],2),x + rawm600.fuse([1 2 6 5],1),-z - rawm600.fuse([1 2 6 5],3),'m');
    H.f2 = fill3(y + rawm600.fuse([1 4 8 5],2),x + rawm600.fuse([1 4 8 5],1),-z - rawm600.fuse([1 4 8 5],3),'m');
    H.f3 = fill3(y + rawm600.fuse([4 3 7 8],2),x + rawm600.fuse([4 3 7 8],1),-z - rawm600.fuse([4 3 7 8],3),'m');
    H.f4 = fill3(y + rawm600.fuse([2 3 7 6],2),x + rawm600.fuse([2 3 7 6],1),-z - rawm600.fuse([2 3 7 6],3),'m');
    H.r1 = plot3(y + rawm600.pylon1(5,2) + rawm600.rotor(:,2),x + rawm600.pylon1(5,1) + rawm600.rotor(:,1),-z - rawm600.pylon1(5,3) - rawm600.rotor(:,3),'k');
    H.r2 = plot3(y + rawm600.pylon1(11,2) + rawm600.rotor(:,2),x + rawm600.pylon1(11,1) + rawm600.rotor(:,1),-z - rawm600.pylon1(11,3) - rawm600.rotor(:,3),'k');
    H.r3 = plot3(y + rawm600.pylon2(5,2) + rawm600.rotor(:,2),x + rawm600.pylon2(5,1) + rawm600.rotor(:,1),-z - rawm600.pylon2(5,3) - rawm600.rotor(:,3),'k');
    H.r4 = plot3(y + rawm600.pylon2(11,2) + rawm600.rotor(:,2),x + rawm600.pylon2(11,1) + rawm600.rotor(:,1),-z - rawm600.pylon2(11,3) - rawm600.rotor(:,3),'k');
    H.r5 = plot3(y + rawm600.pylon3(5,2) + rawm600.rotor(:,2),x + rawm600.pylon3(5,1) + rawm600.rotor(:,1),-z - rawm600.pylon3(5,3) - rawm600.rotor(:,3),'k');
    H.r6 =plot3(y + rawm600.pylon3(11,2) + rawm600.rotor(:,2),x + rawm600.pylon3(11,1) + rawm600.rotor(:,1),-z - rawm600.pylon3(11,3) - rawm600.rotor(:,3),'k');
    H.r7 = plot3(y + rawm600.pylon4(5,2) + rawm600.rotor(:,2),x + rawm600.pylon4(5,1) + rawm600.rotor(:,1),-z - rawm600.pylon4(5,3) - rawm600.rotor(:,3),'k');
    H.r8 = plot3(y + rawm600.pylon4(11,2) + rawm600.rotor(:,2),x + rawm600.pylon4(11,1) + rawm600.rotor(:,1),-z - rawm600.pylon4(11,3) - rawm600.rotor(:,3),'k');
    
else
    
    
    H.wing = fill3(y + rawm600.wing(:,2),x + rawm600.wing(:,1),-z - rawm600.wing(:,3),varargin{1});
    H.p1 = fill3(y + rawm600.pylon1(:,2),x + rawm600.pylon1(:,1),-z - rawm600.pylon1(:,3),varargin{1});
    H.p2 = fill3(y + rawm600.pylon2(:,2),x + rawm600.pylon2(:,1),-z - rawm600.pylon2(:,3),varargin{1});
    H.p3 = fill3(y + rawm600.pylon3(:,2),x + rawm600.pylon3(:,1),-z - rawm600.pylon3(:,3),varargin{1});
    H.p4 = fill3(y + rawm600.pylon4(:,2),x + rawm600.pylon4(:,1),-z - rawm600.pylon4(:,3),varargin{1});
    H.htail = fill3(y + rawm600.htail(:,2),x + rawm600.htail(:,1),-z - rawm600.htail(:,3),varargin{1});
    H.vtail = fill3(y + rawm600.vtail(:,2),x + rawm600.vtail(:,1),-z - rawm600.vtail(:,3),varargin{1});
    H.f1 = fill3(y + rawm600.fuse([1 2 6 5],2),x + rawm600.fuse([1 2 6 5],1),-z - rawm600.fuse([1 2 6 5],3),varargin{1});
    H.f2 = fill3(y + rawm600.fuse([1 4 8 5],2),x + rawm600.fuse([1 4 8 5],1),-z - rawm600.fuse([1 4 8 5],3),varargin{1});
    H.f3 = fill3(y + rawm600.fuse([4 3 7 8],2),x + rawm600.fuse([4 3 7 8],1),-z - rawm600.fuse([4 3 7 8],3),varargin{1});
    H.f4 = fill3(y + rawm600.fuse([2 3 7 6],2),x + rawm600.fuse([2 3 7 6],1),-z - rawm600.fuse([2 3 7 6],3),varargin{1});
    H.r1 = plot3(y + rawm600.pylon1(5,2) + rawm600.rotor(:,2),x + rawm600.pylon1(5,1) + rawm600.rotor(:,1),-z - rawm600.pylon1(5,3) - rawm600.rotor(:,3),varargin{1});
    H.r2 = plot3(y + rawm600.pylon1(11,2) + rawm600.rotor(:,2),x + rawm600.pylon1(11,1) + rawm600.rotor(:,1),-z - rawm600.pylon1(11,3) - rawm600.rotor(:,3),varargin{1});
    H.r3 = plot3(y + rawm600.pylon2(5,2) + rawm600.rotor(:,2),x + rawm600.pylon2(5,1) + rawm600.rotor(:,1),-z - rawm600.pylon2(5,3) - rawm600.rotor(:,3),varargin{1});
    H.r4 = plot3(y + rawm600.pylon2(11,2) + rawm600.rotor(:,2),x + rawm600.pylon2(11,1) + rawm600.rotor(:,1),-z - rawm600.pylon2(11,3) - rawm600.rotor(:,3),varargin{1});
    H.r5 = plot3(y + rawm600.pylon3(5,2) + rawm600.rotor(:,2),x + rawm600.pylon3(5,1) + rawm600.rotor(:,1),-z - rawm600.pylon3(5,3) - rawm600.rotor(:,3),varargin{1});
    H.r6 = plot3(y + rawm600.pylon3(11,2) + rawm600.rotor(:,2),x + rawm600.pylon3(11,1) + rawm600.rotor(:,1),-z - rawm600.pylon3(11,3) - rawm600.rotor(:,3),varargin{1});
    H.r7 = plot3(y + rawm600.pylon4(5,2) + rawm600.rotor(:,2),x + rawm600.pylon4(5,1) + rawm600.rotor(:,1),-z - rawm600.pylon4(5,3) - rawm600.rotor(:,3),varargin{1});
    H.r8 = plot3(y + rawm600.pylon4(11,2) + rawm600.rotor(:,2),x + rawm600.pylon4(11,1) + rawm600.rotor(:,1),-z - rawm600.pylon4(11,3) - rawm600.rotor(:,3),varargin{1});
        
end
