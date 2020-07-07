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

function mat2dat(filename)
% mat2dat -- Converts Makani .mat database file formats into a .dat format.
%
% function mat2dat(filename)
% Converts MATLAB versions of the AVL, DVL, and propeller databases into the
% native simulator format.

[pathstr, name] = fileparts(filename);
data = load(filename);

if isfield(data, 'sampleWing')
  fid = fopen([name, '_rotor.dat'], 'w');
  mat2dat_rotor2(fid, data.sampleWing.propProperties.propData);
  fclose(fid);

  fid = fopen([name, '_avl.dat'], 'w');
  mat2dat_avl(fid, data.sampleWing.aeroProperties.aeroDataAVL);
  fclose(fid);
else
  if isempty(pathstr)
    fid = fopen([name, '.dat'], 'w');
  else
    fid = fopen([pathstr, '/', name, '.dat'], 'w');
  end

  if isfield(data, 'aeroData0')
    mat2dat_avl(fid, data.aeroData0);
  elseif isfield(data, 'aeroData')
    mat2dat_dvl(fid, data.aeroData);
  elseif isfield(data, 'propData')
    mat2dat_rotor(fid, data.propData);
  end

  fclose(fid);
end


% Convert old MATLAB propeller database into .dat format.
function mat2dat_rotor(fid, rotor)

fprintf(fid, '%d %d\n', length(rotor.omegaSpace), length(rotor.UinfSpace));

fprintf(fid, '%0.16g ', rotor.omegaSpace);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', rotor.UinfSpace);
fprintf(fid, '\n');

fprintf(fid, '%0.16g ', rotor.power);
fprintf(fid, '\n');
% I'm not sure why the .mat databases have a weird sign for thrust.
fprintf(fid, '%0.16g ', -rotor.thrust);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', rotor.efficiency);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', rotor.inflow);
fprintf(fid, '\n');


% Convert new MATLAB propeller database into .dat format.
function mat2dat_rotor2(fid, rotor)

fprintf(fid, '%d %d\n', length(rotor.command), length(rotor.velocity));

fprintf(fid, '%0.16g ', rotor.command);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', rotor.velocity);
fprintf(fid, '\n');

fprintf(fid, '%0.16g ', rotor.power);
fprintf(fid, '\n');
% I'm not sure why the .mat databases have a weird sign for thrust.
fprintf(fid, '%0.16g ', -rotor.drag);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', rotor.efficiency);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', rotor.inflow);
fprintf(fid, '\n');


% Convert MATLAB AVL database into .dat format.
function mat2dat_avl(fid, avl)

num_flaps = 0;
fn = fieldnames(avl);
for k = 1:length(fn)
  if ~isempty(strfind(fn{k}, 'cFd')) && ~isempty(str2num(fn{k}(4)))
    num_flaps = max(str2num(fn{k}(4)), num_flaps);
  end
end

fprintf(fid, '%d %d %d\n', length(avl.alpha), length(avl.beta), num_flaps);

fprintf(fid, '%0.16g ', avl.alpha);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', avl.beta);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', avl.df);
fprintf(fid, '\n');
if isfield(avl, 'Cw0s')
  fprintf(fid, '%0.16g ', avl.Cw0s);
  fprintf(fid, '\n');
else
  fprintf(fid, '%0.16g ', [0, 0, 0]);
  fprintf(fid, '\n');
end

fprintf(fid, '%0.16g ', squeeze(avl.cF(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cF(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cF(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cM(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cM(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cM(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFdOmega1(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFdOmega1(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFdOmega1(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMdOmega1(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMdOmega1(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMdOmega1(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFdOmega2(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFdOmega2(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFdOmega2(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMdOmega2(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMdOmega2(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMdOmega2(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFdOmega3(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFdOmega3(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFdOmega3(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMdOmega3(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMdOmega3(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMdOmega3(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd1(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd1(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd1(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd1(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd1(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd1(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd2(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd2(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd2(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd2(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd2(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd2(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd3(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd3(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd3(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd3(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd3(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd3(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd4(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd4(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd4(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd4(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd4(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd4(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd5(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd5(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cFd5(3, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd5(1, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd5(2, :, :))');
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', squeeze(avl.cMd5(3, :, :))');
fprintf(fid, '\n');
if isfield(avl, 'cFd8')
  fprintf(fid, '%0.16g ', squeeze(avl.cFd6(1, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cFd6(2, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cFd6(3, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cMd6(1, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cMd6(2, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cMd6(3, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cFd7(1, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cFd7(2, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cFd7(3, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cMd7(1, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cMd7(2, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cMd7(3, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cFd8(1, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cFd8(2, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cFd8(3, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cMd8(1, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cMd8(2, :, :))');
  fprintf(fid, '\n');
  fprintf(fid, '%0.16g ', squeeze(avl.cMd8(3, :, :))');
  fprintf(fid, '\n');
end

% Convert MATLAB DVL database into .dat format.
function mat2dat_dvl(fid, dvl)

% This is a placeholder for the Reynolds number that will trigger a runtime
% crash in the simulator.
fprintf(fid, '%0.16g\n', -1.0);

fprintf(fid, '%d %d %d %d %d %d %d\n', ...
    length(dvl.alphas), length(dvl.betas), ...
    length(dvl.deltas1), length(dvl.deltas2), length(dvl.deltas3), ...
    length(dvl.deltas4), length(dvl.deltas5));

fprintf(fid, '%0.16g ', dvl.alphas);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', dvl.betas);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', dvl.deltas1);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', dvl.deltas2);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', dvl.deltas3);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', dvl.deltas4);
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', dvl.deltas5);
fprintf(fid, '\n');

fprintf(fid, '%0.16g ', permute(dvl.cFMain, [3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.SwMain(1:6, :, :), [3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.SwMain(7:12, :, :), [3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.SwMain(13:18, :, :), [3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dcF1, [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dcF2, [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dcF3, [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dcF4, [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dcF5, [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw1(1:6, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw2(1:6, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw3(1:6, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw4(1:6, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw5(1:6, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw1(7:12, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw2(7:12, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw3(7:12, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw4(7:12, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw5(7:12, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw1(13:18, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw2(13:18, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw3(13:18, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw4(13:18, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
fprintf(fid, '%0.16g ', permute(dvl.dSw5(13:18, :, :, :), [4, 3, 2, 1]));
fprintf(fid, '\n');
