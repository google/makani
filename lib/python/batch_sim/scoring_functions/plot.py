# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Plot the time-series of scoring events."""

from bokeh import models as bokeh_models
from bokeh import plotting as bokeh_plotting


def PlotEvents(results, output_html):
  """Plot the time-series of scoring events."""
  all_categories = sorted(results.keys())
  categories = []
  references = []
  for c in all_categories:
    if 'Loop Angle' in c:
      references.append(c)
    else:
      categories.append(c)

  category_index = {}
  for i, c in enumerate(references + categories):
    category_index[c] = i

  scoring_color = {'warning': 'orange', 'error': 'red'}
  reference_color = {'warning': 'blue', 'error': 'green'}

  point_x = []
  point_y = []
  colors = []

  max_length = 0
  x_min = float('inf')
  x_max = float('-inf')
  for check_name, log_result in results.iteritems():
    result = log_result.values()[0]
    colormap = reference_color if check_name in references else scoring_color
    for level, color in colormap.iteritems():
      if level in result:
        sections = result[level]['sections']
        for section in sections:
          max_length = max(max_length, section[1])
          for i in range(section[0], section[1]):
            point_x.append(i)
            x_min = min(x_min, i)
            x_max = max(x_max, i)
            name = check_name
            point_y.append(category_index[name])
            colors.append(color)

  p = bokeh_plotting.figure(
      title='Orange is 0 < score < 100, and Red is score > 100', width=1800)
  p.yaxis.ticker = range(len(category_index))
  p.yaxis.major_label_overrides = {v: k for k, v in category_index.iteritems()}

  # Turn off ticks and labels on X-axis.
  p.xaxis.major_label_text_font_size = '0pt'
  p.xaxis.major_tick_line_color = None
  p.xaxis.minor_tick_line_color = None

  p.xgrid.ticker = bokeh_models.FixedTicker(
      ticks=range(0, max(1, max_length), max(1, max_length/60)))
  p.ygrid.ticker = bokeh_models.FixedTicker(
      ticks=range(max(1, len(category_index))))

  p.circle(point_x, point_y,
           color=colors, fill_alpha=0.2, size=3)
  divider_y = len(references) - 0.5
  p.line([x_min, x_max], [divider_y, divider_y], color='grey', line_width=2)
  # TODO: Toolbar can't be shown due to HTML security protocol.
  p.toolbar_location = None
  bokeh_plotting.output_file(output_html, title='Log events')

  bokeh_plotting.save(p)
