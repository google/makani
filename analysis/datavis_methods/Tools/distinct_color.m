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

function RGB_array = distinct_color(n)
% distinct_color: get a set of ditinct colors for plotting
% RGB_array = distinct_color(n)
% Uses the colorset in reference for upto 26 colors,
% defaults to colorcube if more than 26 colors are requested.
% Reference: http://epub.wu.ac.at/1692/1/document.pdf
%
% Arguements--
% n: number of distinct colors
%
% Output Values--
% RGB_array: [n x 3] array containing RGB values of colors

master_set = [0 0 0
1 0 103
213 255 0
255 0 86
158 0 142
14 76 161
255 229 2
0 95 57
0 255 0
149 0 58
255 147 126
164 36 0
0 21 68
145 208 203
98 14 0
107 104 130
0 0 255
0 125 181
106 130 108
0 174 126
194 140 159
190 153 112
0 143 156
95 173 78
255 0 0
255 0 246
255 2 157
104 61 59
255 116 163
150 138 232
152 255 82
167 87 64
1 255 254
255 238 232
254 137 0
189 198 255
1 208 255
187 136 0
117 68 177
165 255 210
255 166 254
119 77 0
122 71 130
38 52 0
0 71 84
67 0 44
181 0 255
255 177 103
255 219 102
144 251 146
126 45 210
189 211 147
229 111 254
222 255 116
0 255 120
0 155 255
0 100 1
0 118 255
133 169 0
0 185 23
120 130 49
0 255 198
255 110 65
232 94 190]/255;

if n <= 26
  RGB_array = master_set(1:1:n,:);
else
  RGB_array = colorcube(n);
end