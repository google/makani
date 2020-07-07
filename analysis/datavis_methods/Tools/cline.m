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

% This function plots a 3D line (x,y,z) encoded with scalar color data (c)
% using the specified colormap (default=jet);
%
% SYNTAX: h=cline(x,y,z,c,colormap);
%
% DBE 09/03/02

% modified from https://www.mathworks.com/matlabcentral/fileexchange/14677-cline


%% example usage:
% 
% Posn.X=ControllerA.message.state_est.Xg.x;
% Posn.Y=ControllerA.message.state_est.Xg.y;
% Posn.Z=ControllerA.message.state_est.Xg.z;
% 
% figure
% cline(Posn.X,-Posn.Y,-Posn.Z,motor.mech_power_total./1000,'jet');
% xlabel('Position West of GS, m')
% ylabel('Position South of GS, m')
% zlabel('Altitude above GS, m')
% title('Total Mechanical Power, kW')
% axis equal
% % zlim([0 450])
% view([90,0])

%%

function h=cline(x,y,z,c,cmap,cmin,cmax)

if nargin==0  % Generate sample data...
  x=linspace(-10,10,101);
  y=2*x.^2+3;
  z=sin(0.1*pi*x);
  c=exp(z);
  w=z-min(z)+1;
  cmap='jet';
elseif nargin<4
  fprintf('Insufficient input arguments\n');
  return;
elseif nargin==4
  cmap='jet';
elseif nargin==5
    cmin=min(c);
    cmax=max(c);
end

cmap=colormap(cmap);                      % Set colormap
% yy=linspace(min(c),max(c),size(cmap,1));  % Generate range of color indices that map to cmap
yy=linspace(cmin,cmax,size(cmap,1));  % Generate range of color indices that map to cmap
cm = spline(yy,cmap',c);                  % Find interpolated colorvalue
cm(cm>1)=1;                               % Sometimes iterpolation gives values that are out of [0,1] range...
cm(cm<0)=0;

% Lot line segment with appropriate color for each data pair...
  for i=1:length(z)-1
    h(i)=line([x(i) x(i+1)],[y(i) y(i+1)],[z(i) z(i+1)],'color',[cm(:,i)],'LineWidth',3);
  end
set(gca, 'CLim', [cmin, cmax]);
colorbar;
return
