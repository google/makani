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

% AddPlotInterestingTimes function to add markers to current plot where are
% unusually large 1st and 2nd derivatives in the data

% Usage:
%     figure('name','tether tension')
%     plot(d.time,d.tether_tension)
%     AddPlotInterestingTimes(3)
%     grid
%     title([d.title,':  Unfiltered Tether Tension'])
%     xlabel('time (sec)')
%     ylabel('tension (N)')

% Description:
%   call this after making a plot, and give it how many 'peak_std' you want
%   it to point out. 3 seems to work well - only gives you the top half
%   dozen or fewer instances of a flight. Use, say, 2 to be more sensitive
%   or use more to only catch the most unusual cases.
% 
%   This extracts the x and y data from the current plot, filters it by a
%   factor of 10, differentiates twice. Then determines what variation in
%   these derivitives might be 'normal' by finding the largest peaks in a
%   10s window. Then it uses the mean and std of this variation to find the
%   extremes, determined by the number of standard deviations passed to it
%   (peak_std), and adds a circle to original data where the 1st derivative
%   was extreme, and a diamond to the plot where the 2nd derivative was
%   extreme. A 1s window is used for the extremes so the point of most
%   interest may not be at the exact points plotted
%
% Inputs:
%   f_mph_std: number of stds from the mean of the 'main peaks' to define 'interesting'
%
% Outputs:
%   none (although it will spit out an warning if no peaks get detected,
%   which just means that there were no extreme outliers in the 1st or 2nd
%   derivative so it generally appears normal; you can use a smaller
%   f_mph_std if you want to catch more instances)
% 
% Future Work:
%   - maybe tweak algorithm if needed to better work for a wide range of
%   data, or allow more parameters to be passed to tweak the process (eg,
%   the amount of filtering, mpd1 and 2, etc
%   - maybe improve how the colors for the markers are selected so they
%   better stand out when added to a line of the same color
%   - maybe incorporate peaks of the original data too? (fairly easy to
%   identify visually though)
%   - maybe return an array of the time instances found


function AddPlotInterestingTimes(f_mph_std)
% This takes the current figure, filters all the signals, and places a
% marker at every time where the first or second derivative of this
% filtered signal exceeds a set fraction of standard deviations from the
% majority of the peaks that generally trend w/ each loop

% f_mph_std=3; % range of stds from the mean of the main peaks to define 'interesting'; 3 seems to work well

hold on
% extra data from current figure
D=get(gca,'Children'); %get the handle of the line object
XData=get(D,'XData'); %get the x data
YData=get(D,'YData'); %get the y data

% for each set of data, 
for ii=1:size(XData,1)
    if size(XData,1)>1 % data is returned as a cell
        x= XData{ii};
        y= YData{ii};
    else % data returned as vector
        x= XData;
        y= YData;
    end
    sample_freq = 1/nanmean(diff(x)); 
    cuttoff_freq = sample_freq/10; % filtering by factor of 10 seems to give reasonable results
    [b,a] = butter(1,cuttoff_freq/(sample_freq/2));
    y_f= filtfilt(b, a, y); % filter the signal
    y_f_v= [nan, diff(y_f)./diff(x)]; % differentiate
    y_f_v_f= [nan, filtfilt(b, a, [y_f_v(2:end)])]; % filter the 1st derivative
    y_f_a= [nan, diff(y_f_v_f)./diff(x)]; % differentiate again

    mpd1 = 10; % MinPeakDistance for setting the 'nominal peaks', same units as x, using ~10s seems to work well to catch the min and max for most peaks
    mpd2 = 1; % MinPeakDistance for picking out extremes, same units as x, using ~1s seems to work well to catch the min and max for most peaks
    [vPKS,vLOCS] = findpeaks(abs(y_f_v),x,'MinPeakDistance',mpd1); % find the max and min local peaks of the first derivative that are at least dt_win apart
    mph = mean(vPKS)+std(vPKS)*f_mph_std; % figure out how much those peak vary, and set a threashold that is f_mph_std *std from the mean
    [vPKS,vLOCS]= findpeaks(abs(y_f_v),x,'MinPeakHeight',mph,'MinPeakDistance',mpd2); % find all extreme peaks in the 1st derivative
    [aPKS,aLOCS] = findpeaks(abs(y_f_a),x,'MinPeakDistance',mpd1); % repeat for 2nd deriv
    mph = mean(aPKS)+std(aPKS)*f_mph_std; % figure out how much those peak vary, and set a threashold that is f_mph_std *std from the mean
    [aPKS,aLOCS]= findpeaks(abs(y_f_a),x,'MinPeakHeight',mph,'MinPeakDistance',mpd2);
    plot(vLOCS,interp1(x,y,vLOCS),'or',aLOCS,interp1(x,y,aLOCS),'dm')

    if 0 % for debugging or fine-tuning parameters
        % make a new figure with the original data and the derivatives and
        % overlay the detected peaks
        figure 
        ax(1)= subplot(3,1,1);
        plot(x,y,'.c',x,y_f,'-b',vLOCS,ones(size(vLOCS))*mean(y),'or',aLOCS,ones(size(aLOCS))*mean(y),'sm')
        ax(2)= subplot(3,1,2);
        plot(x,y_f_v,'-b')
        ax(3)= subplot(3,1,3);
        plot(x,y_f_a,'-b')
        linkaxes(ax, 'x');
        keyboard
    end
end

hold off
end