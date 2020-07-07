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

"""Interactive analysis utils for Jupyter Notebooks.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from IPython.display import clear_output
from IPython.display import display
import ipywidgets as widgets
from makani.analysis.util.simdata_analysis import compare_simdata
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

sns.set_style('whitegrid')

score_comparison = []
_SCORE_THR = 1.0


def interactive_scatterplot(simdata, table, scores=None,
                            kws_scatterplot=None, kws_distplot=None,
                            kws_hexbin=None):
  """Interactive scatterplot of a score vs. another score.

  Scores for each axis are selectable using dropdown menus. Options are provided
  to draw a 2D histogram and lines indicating the score thresholds.

  From a jupyter notebook it may be necessary to use the magic command
  '%matplotlib notebook' for a correct visualization.

  Args:
    simdata: simdata.SimData object containing the data.
    table: Table to use.
    scores: List of scores to plot. If None, all of the scores in simdata are
        used.
    kws_scatterplot: Keywords passed to the seaborn.scatterplot function.
    kws_distplot: Keywords passed to the seaborn.distplot function, which draws
        the marginal distributions.
    kws_hexbin: Keywords pased to the matplotlib.pyplot.hexbin function, which
        draws the 2D histogram.
  """
  if scores is None:
    score_list = simdata.get_score_list()
  else:
    # Validate the scores.
    score_list = [simdata.get_score_info(sc)['name'] for sc in scores]

  if kws_scatterplot is None:
    kws_scatterplot = {}

  if kws_distplot is None:
    kws_distplot = {'kde': False, 'bins': 50}
  else:
    if 'kde' not in kws_distplot:
      kws_distplot['kde'] = False
    if 'bins' not in kws_distplot:
      kws_distplot['bins'] = 50

  if kws_hexbin is None:
    kws_hexbin = {'gridsize': (20, 20), 'cmap': 'Purples'}
  else:
    if 'gridsize' not in kws_hexbin:
      kws_hexbin['gridsize'] = (20, 20)
    if 'cmap' not in kws_hexbin:
      kws_hexbin['cmap'] = 'Purples'

  hist_2d_cb = widgets.Checkbox(value=False,
                                description='2D histogram',
                                disabled=False)
  score_thr_cb = widgets.Checkbox(value=False,
                                  description='Score threshold',
                                  disabled=False)

  horiz = widgets.Dropdown(options=score_list,
                           value=score_list[0],
                           description='X axis score:',
                           disabled=False,
                           layout=widgets.Layout(width='6in'))
  vert = widgets.Dropdown(options=score_list,
                          value=score_list[0],
                          description='Y axis score:',
                          disabled=False,
                          layout=widgets.Layout(width='6in'))

  display(widgets.HBox([hist_2d_cb, score_thr_cb]))
  display(vert)

  x0_var_data = simdata.get_score_data(score_list[0], table, verbose=False).data
  y0_var_data = simdata.get_score_data(score_list[0], table, verbose=False).data
  g = sns.jointplot(x0_var_data, y0_var_data, space=0, ratio=7)
  g.ax_joint.set_xlabel(score_list[0])
  g.ax_joint.set_ylabel(score_list[0])
  plt.tight_layout()

  display(horiz)

  def handle_change():
    """Handles changes in the interface."""
    score_x = horiz.value
    score_y = vert.value
    x_var_data = simdata.get_score_data(score_x, table, verbose=False).data
    y_var_data = simdata.get_score_data(score_y, table, verbose=False).data

    g.ax_joint.clear()
    g.ax_marg_x.clear()
    g.ax_marg_y.clear()
    g.x = x_var_data
    g.y = y_var_data
    if hist_2d_cb.value:
      g.plot_joint(plt.hexbin, **kws_hexbin)
    else:
      g.plot_joint(sns.scatterplot, **kws_scatterplot)
    g.plot_marginals(sns.distplot, **kws_distplot)
    g.ax_joint.set_xlabel(score_x)
    g.ax_joint.set_ylabel(score_y)
    g.ax_marg_x.set_yticks([])
    g.ax_marg_y.set_xticks([])
    g.ax_marg_x.grid('off')
    g.ax_marg_y.grid('off')
    if score_thr_cb.value:
      g.ax_joint.axhline(_SCORE_THR, linewidth=2, color='k', linestyle='--')
      g.ax_joint.axvline(_SCORE_THR, linewidth=2, color='k', linestyle='--')
    g.ax_joint.grid(True)
    g.fig.show()

  horiz.on_trait_change(handle_change, name='value')
  vert.on_trait_change(handle_change, name='value')
  hist_2d_cb.on_trait_change(handle_change, name='value')
  score_thr_cb.on_trait_change(handle_change, name='value')
  handle_change()


def interactive_cdf(simdata):
  """Plots score CDFs.

  Allows selection of score and table combinations and dsisplays the CDFs of the
  selected pairs.

  Args:
    simdata: simdata.SimData object containing the data.
  """
  score_list = ['{0}: {1}'.format(idx, sn) for (idx, sn) in
                enumerate(simdata.get_score_list())]
  table_list = ['{0}: {1}'.format(idx, tn) for (idx, tn) in
                enumerate(simdata.get_table_list())]
  score_select = widgets.Select(options=score_list,
                                value=None,
                                rows=10,
                                description='Scores:',
                                layout=widgets.Layout(width='45%'),
                                disabled=False)
  table_select = widgets.Select(options=table_list,
                                value=None,
                                rows=10,
                                description='Tables:',
                                layout=widgets.Layout(width='45%'),
                                disabled=False)
  score_table_select = widgets.Select(options=[],
                                      rows=5,
                                      description='Score+table:',
                                      layout=widgets.Layout(width='90%'),
                                      disabled=False)
  add_button = widgets.Button(description='Add',
                              disabled=False,
                              button_style='success',
                              layout=widgets.Layout(width='20%'),
                              tooltip='Add score/table combination',
                              icon='fa-arrow-down')
  del_button = widgets.Button(description='Remove',
                              disabled=False,
                              button_style='warning',
                              layout=widgets.Layout(width='20%'),
                              tooltip='Remove score/table combination',
                              icon='fa-arrow-up')
  reset_button = widgets.Button(description='Reset',
                                disabled=False,
                                button_style='danger',
                                layout=widgets.Layout(width='10%'),
                                tooltip='Reset',
                                icon='fa-home')
  score_thr_cb = widgets.Checkbox(value=False,
                                  description='Score threshold',
                                  disabled=False)
  enable_readouts_cb = widgets.Checkbox(value=False,
                                        description='Enable CDF reading',
                                        disabled=False)
  cdf_slider = widgets.FloatSlider(min=0, max=1, step=0.001, value=0,
                                   orientation='vertical',
                                   continuous_update=False,
                                   readout_format='.3f',
                                   layout=widgets.Layout(height='3.5in',
                                                         align_self='center'))
  cdf_readout = widgets.Select(options=[],
                               value=None,
                               rows=10,
                               description='',
                               layout=widgets.Layout(width='1in',
                                                     align_self='center'),
                               disabled=True)
  out = widgets.Output(layout={'width': '7in', 'height': '5in'})

  # Display the widgets.
  display(widgets.VBox([widgets.HBox([score_select, table_select]),
                        widgets.HBox([add_button, del_button, reset_button],
                                     layout=widgets.Layout(
                                         justify_content='center')),
                        score_table_select,
                        widgets.HBox([score_thr_cb, enable_readouts_cb])]))

  display(widgets.HBox([out, cdf_slider, cdf_readout]))

  # Create empty plot.
  with out:
    fig, _ = plt.subplots()
    fig.set_size_inches(6, 4)
    plt.xlabel('Score values')
    plt.ylabel('CDF')
    plt.tight_layout()
    plt.show()

  var_list = []

  def plot_ecdfs():
    """Plot ecdfs."""
    with out:
      clear_output(wait=True)
      fig, ax = plt.subplots()
      fig.set_size_inches(6, 4)
      plt.xlabel('Score values')
      plt.ylabel('CDF')
    cdf_readout.options = []
    cdf_readout.value = None

    if not var_list:
      return

    with out:
      cdf_list = []
      for i, var in enumerate(var_list):
        vardata = var[2]
        xs, ecdf_mean_bounds = vardata.ecdf_data()
        ax.plot(xs, ecdf_mean_bounds[0, :], label=str(i+1))
        if enable_readouts_cb.value:
          cdf_list.append('{0} : {1:.3f}'.format(
              i+1, vardata.percentile(cdf_slider.value*100.).mean))
      if score_thr_cb.value:
        ax.axvline(_SCORE_THR, linewidth=2, color='k', linestyle='--')
      if enable_readouts_cb.value:
        ax.axhline(cdf_slider.value, linewidth=2, color='k', linestyle='--')
        cdf_readout.options = cdf_list
        cdf_readout.value = None
      plt.xlabel(vardata.variable_info['name'])
      plt.ylabel('CDF')
      plt.title(vardata.table_info['title'])
      plt.yticks(np.linspace(0, 1, 11))
      plt.ylim(0, 1)
      plt.legend()
      ax.grid(True)
      plt.show()

  def refresh_score_table():
    """Refresh data in score table select."""
    var_str_list = []
    for i, var in enumerate(var_list):
      score_name = simdata.get_score_info(var[0], verbose=False)['name']
      table_name = simdata.get_table_info(var[1], verbose=False)['title']
      var_name = '{0} - "{1}" : "{2}"'.format(i+1, score_name, table_name)
      var_str_list.append(var_name)
    score_table_select.options = var_str_list
    score_table_select.value = None
    plot_ecdfs()

  def handle_add(_):
    """Handle pressing the Add button."""
    if score_select.value is None or table_select.value is None:
      return
    score_idx = simdata.get_score_info(score_select.value.split(': ', 1)[-1],
                                       verbose=False)['index']
    table_idx = simdata.get_table_info(table_select.value.split(': ', 1)[-1],
                                       verbose=False)['index']
    score_vardata = simdata.get_score_data(score_idx, table_idx, verbose=False)
    var_list.append((score_idx, table_idx, score_vardata))
    refresh_score_table()

  def handle_remove(_):
    """Handle pressing the Remove button."""
    if score_table_select.value is None:
      return
    var_list.pop(score_table_select.index)
    refresh_score_table()

  def handle_reset(_):
    """Handle pressing the Reset button."""
    var_list[:] = []
    refresh_score_table()

  add_button.on_click(handle_add)
  del_button.on_click(handle_remove)
  reset_button.on_click(handle_reset)
  score_thr_cb.on_trait_change(plot_ecdfs, name='value')
  enable_readouts_cb.on_trait_change(plot_ecdfs, name='value')
  cdf_slider.on_trait_change(plot_ecdfs, name='value')


def interactive_simdata_comparison(prior_simdata, post_simdata,
                                   score_thr=0.999, p_val_thr=0.05):
  """Interactive simulation data comparison.

  From a jupyter notebook it may be necessary to use the magic command
  '%matplotlib notebook' for a correct visualization.

  Args:
    prior_simdata: SimData object with the prior data.
    post_simdata: SimData object with the posterior data.
    score_thr: Score threshold at which to evaluate the probabilities.
    p_val_thr: P-value threshold.
  """
  table_list = prior_simdata.get_table_list()
  table_select = widgets.Select(options=table_list,
                                value=None,
                                rows=10,
                                description='Tables:',
                                layout=widgets.Layout(width='40%'),
                                disabled=False)
  get_button = widgets.Button(description='Get comparison',
                              disabled=False,
                              button_style='',
                              layout=widgets.Layout(width='20%',
                                                    align_self='center'),
                              tooltip='Add score/table combination',
                              icon='fa-arrow-right')
  score_select = widgets.Select(options=[],
                                value=None,
                                rows=10,
                                description='Scores:',
                                layout=widgets.Layout(width='40%'),
                                disabled=False)

  comparison_table_out = widgets.Output(layout={'width': '30%',
                                                'height': '5in'})
  comparison_cdf_out = widgets.Output(layout={'width': '70%',
                                              'height': '5in'})

  display(widgets.HBox([table_select, get_button, score_select]))
  display(widgets.HBox([comparison_table_out, comparison_cdf_out]))

  def handle_compare(_):
    """Handle click on compare button."""
    global score_comparison
    if table_select.value is None:
      return
    table = table_select.value
    score_select.disabled = True
    score_select.options = []
    score_select.value = None
    score_comparison = compare_simdata.get_simdata_comparison(prior_simdata,
                                                              post_simdata,
                                                              table, score_thr,
                                                              p_val_thr)
    if score_comparison:
      score_select.options = [sc['prior_score'] for sc in score_comparison]
      score_select.value = score_comparison[0]['prior_score']
      score_select.disabled = False
    else:
      # Comparison is empty.
      score_select.options = ['No statistically significant differences.']
      score_select.value = None
      score_select.disabled = True

  def handle_select(_):
    """Handle score select."""
    if score_select.value is None:
      with comparison_cdf_out:
        clear_output(wait=False)
      with comparison_table_out:
        clear_output(wait=False)
      return

    score = score_select.value
    table = table_select.value
    prior_vardata = prior_simdata.get_score_data(score, table, verbose=False)
    post_vardata = post_simdata.get_score_data(score, table, verbose=False)

    with comparison_cdf_out:
      clear_output(wait=True)
      fig, ax = plt.subplots()
      fig.set_size_inches(6, 4)
      plt.xlabel(score)
      plt.ylabel('CDF')

      xs_prior, ecdf_mean_bounds_prior = prior_vardata.ecdf_data()
      ax.plot(xs_prior, ecdf_mean_bounds_prior[0, :], label='Prior')
      xs_post, ecdf_mean_bounds_post = post_vardata.ecdf_data()
      ax.plot(xs_post, ecdf_mean_bounds_post[0, :], label='Posterior')
      plt.legend()
      plt.title(table, fontsize=10)
      plt.grid(True)

    with comparison_table_out:
      if score_comparison:
        clear_output(wait=True)
        df = pd.DataFrame(score_comparison[score_select.index],
                          index=[0]).transpose()
        df.columns = ['Comparison']
        display(df)

  get_button.on_click(handle_compare)
  score_select.on_trait_change(handle_select, name='value')
