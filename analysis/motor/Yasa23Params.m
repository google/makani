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

function params = Yasa23Params(source)
% Yasa23Params -- Return electrical parameters of a Yasa 2.3 motor.
%
% [params] = Yasa23Params(reference)
%
% Arguments:
%
% reference: String specifying which source to use for motor parameters:
%            'Yasa': (default) Use Yasa's 2016-06-15 engineering report.
%
% Return Values:
%
% params: Struct containing motor electrical and sensor properties.
  persistent c_lambda_yasa

  % Set default source if none is specified.
  if (nargin < 1)
    source = 'Yasa';
  end

  nppe = 15;  % Number of electrical pole pairs.
  npps = 5;   % Number of sensor pole pairs.

  switch (lower(source))
    case {'yasa'}
      if (isempty(c_lambda_yasa))
        % The following points were taken from the Yasa 2.3 Kt map in figure 8
        % of https://goo.gl/3vmQG8 using WebPlotDigitizer. The phase current
        % (column 1) is in RMS Amps while torque is in Nm.
        Kt_points = [  0.00,    0.00
                      20.11,  104.77
                      40.06,  208.00
                      60.00,  309.77
                      80.00,  409.36
                     100.00,  505.32
                     120.00,  597.64
                     139.94,  686.32
                     159.94,  768.83
                     179.94,  845.89
                     199.89,  918.22
                     219.89,  984.37
                     239.89, 1044.34];

        i_pk = Kt_points(:, 1) * sqrt(2);  % Convert to Amps phase peak.
        tau = Kt_points(:, 2);

        % Evaluate the least squares coefficients using the model
        %
        %   tau = 3/2 * nppe * i_pk * lambda(i_pk)
        %       = 3/2 * nppe * i_pk * (c(1) + c(2) * i_pk + c(3) * i_pk^2)
        %
        % The c(0) term is ignored because there shouldn't be any
        % electromechanical torque when i_pk = 0.
        A = [i_pk, i_pk.^2, i_pk.^3];
        c_lambda_yasa = (A \ tau) / (1.5 * nppe);

        % We treat i_pk as purely q-current in calculating torque which gets rid
        % effects from saliency. However, it is worth noting:
        % (1) It is not completely clear what i_pk is from Yasa's documentation;
        %     it is possible that they assume the current vector is following
        %     the maximum torque per Amp line which deviates slightly from the
        %     q-axis due to saliency effects.
        % (2) It is not clear how the flux linkage nonlinearities are affected
        %     by flux weakening; the flux linkage, lambda(x), may want to be
        %     parameterized in terms of lambda(iq) or lambda(i_pk).
        % Both of these issues are third order effects, but will affect the
        % estimated torque by a couple percent.
      end
      c_lambda = c_lambda_yasa;

      % Ld and Lq inductances are quoted for operating conditions of
      % id = -50 A (RMS) and iq = +50 A (RMS). However, there is a decent amount
      % of uncertainty regarding the accuracy of these numbers and how they vary
      % with increased current. Several tables in the engineering report suggest
      % Ld < Lq and a number of experiments at Makani suggest operational
      % inductances closer to 0.85 mH.
      %
      % Updated Ld to 0.85 mH based on:
      % https://docs.google.com/presentation/d/1X_3ywQN0HbWqDKgM3Yv1ylfGTDBNpIUw8asGWglLYAA/edit#slide=id.g1fac39201c_0_148
      Ld = 0.85e-3;  % [H]
      Lq = 0.9e-3;  % [H]

    % TODO: Add option using 1x1 dyno data.
    otherwise
      assert(false, 'Unable to find motor parameter source.');
  end

  params = struct('Ld', Ld, ...            % [H].
                  'Lq', Lq, ...            % [H].
                  'lambda', c_lambda, ...  % [Nm / A_phpk].
                  'Rs', 0.103, ...         % [Ohm].
                  'npp_elec', nppe, ...    % [#].
                  'npp_sens', npps);       % [#].
end
