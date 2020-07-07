/**
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Legend dimensions.
const LEGEND_HEADER_HEIGHT = 30;
const LEGEND_CELL_HEIGHT = 35;
const LEGEND_CELL_WIDTH = LEGEND_CELL_HEIGHT;
const LEGEND_HEIGHT = LEGEND_HEADER_HEIGHT + LEGEND_CELL_HEIGHT;

// Table cell dimensions.
const TABLE_CELL_WIDTH = 50;
const TABLE_CELL_HEIGHT = TABLE_CELL_WIDTH;

// Special score value.
const SIM_FAILURE = 'sim_failure';

// The color map. Ideally, this would probably be a member of an object that
// more fully describes the score table layout.
let globalColorMap = null;

// The batch statistics list. This gets assigned when the scores are listed for
// the first time.
let batchStatistics = null;

// Keep track of state for the dynamic plots.
let scorePlotData = {};

/**
 * Wraps SVG text to a specified width. This is taken from
 * https: *bl.ocks.org/mbostock/7555321.
 *
 * @param {d3.selection} text The text elements to wrap, provided implicitly
 *     via d3.selection.call.
 * @param {number} width The width to which the text will be wrapped.
 */
function wrapText(text, width) {
  text.each(function() {
    let text = d3.select(this), words = text.text().split(/\s+/).reverse();
    let word = [];
    let line = [];
    let lineNumber = 0;
    let lineHeight = 1.1;  // ems
    let x = text.attr("x");
    let y = text.attr("y");
    let dy = parseFloat(text.attr("dy"));
    let tspan = text.text(null).append("tspan").attr("x", x).attr("y", y).attr(
        "dy", dy + "em");
    while (word = words.pop()) {
      line.push(word);
      tspan.text(line.join(" "));
      if (tspan.node().getComputedTextLength() > width) {
        line.pop();
        tspan.text(line.join(" "));
        line = [word];
        tspan = text.append("tspan")
                    .attr("x", x)
                    .attr("y", y)
                    .attr("dy", ++lineNumber * lineHeight + dy + "em")
                    .text(word);
      }
    }
  });
}

/**
 * Draws the legend.
 *
 * @param {string} elt ID of the containing element. Should have one member of
 *     class 'legend'.
 * @param {d3.scaleQuantize} colorMap Quantize scale mapping scores to colors.
 */
function drawLegend(elt, colorMap) {
  let numColors = colorMap.range().length;

  // Note that there are two legend cells for the last color.
  let colorDomain = [];
  for (let i = 0; i <= numColors; i++) {
    colorDomain.push(i / numColors);
  }
  let legendWidth = colorDomain.length * LEGEND_CELL_WIDTH;

  let legend = d3.select(elt)
                   .selectAll('.legend')
                   .append('svg')
                   .attr('width', legendWidth)
                   .attr('height', LEGEND_HEIGHT)
                   .attr('class', 'legend');

  // Left label: "Good".
  legend.append('text')
      .attr('x', LEGEND_CELL_WIDTH / 2)
      .attr('y', LEGEND_HEADER_HEIGHT / 2)
      .style('text-anchor', 'middle')
      .text('Good');

  // Right label: "Bad".
  legend.append('text')
      .attr('x', legendWidth - LEGEND_CELL_WIDTH / 2)
      .attr('y', LEGEND_HEADER_HEIGHT / 2)
      .style('text-anchor', 'middle')
      .text('Bad');

  // Legend cells.
  legend.selectAll('.cell')
      .data(colorDomain)
      .enter()
      .append('svg:rect')
      .attr('class', 'cell')
      .attr('x', function(d, i) { return i * LEGEND_CELL_WIDTH; })
      .attr('y', LEGEND_HEADER_HEIGHT)
      .attr('width', LEGEND_CELL_WIDTH)
      .attr('height', LEGEND_CELL_HEIGHT)
      .style('stroke', '#555')
      .style('stroke-width', '1px')
      .style('fill', function(d) { return colorMap(d); });

  // Cell labels.
  legend.selectAll('.label')
      .data(colorDomain)
      .enter()
      .append('svg:text')
      .attr('x', function(d, i) { return (i + 0.5) * LEGEND_CELL_WIDTH; })
      .attr('y', LEGEND_HEADER_HEIGHT + LEGEND_CELL_HEIGHT / 2)
      .attr('text-anchor', 'middle')
      .attr('dy', '.35em')
      .text(function(d) { return Math.round(100 * d); });
}

/*
 * Computes the root sum square (RSS) of the elements in an array.
 * @param {Array<number>} array Array of numbers.
 * @return {number} Root sum square of the numbers.
 */
function rootSumSquare(array) {
  let out = 0;
  for (let i = 0; i < array.length; i++) {
    out += Math.pow(array[i], 2);
  }
  return Math.sqrt(out);
}

/**
 * Aggregates a list of scores into a single score.
 * See documentation: docs.google.com/document/d/1XPmwuK0Ija6gv3cUtX5LEocv2QwPCLN8yXXssM8R6Fg
 * @param {Array<number>} jobData JS Object containing an array of scores.
 * @return {number} Aggregated score.
 */
function aggregate(jobData) {
  if (!jobData['sim_success']) {
    return SIM_FAILURE;
  }

  let activeCrashScores = [];
  let activeQualityScores = [];
  let activeQualityScoresSeverity = [];
  let activeQualityWeightedScores = [];

  d3.select('#active_scores').selectAll('input').each(function(d, i) {
    if (d3.select(this).property('checked')) {
      let systemLabel = d3.select(this).data()["0"].system_labels;
      let severity = d3.select(this).data()["0"].severity;
      let score = jobData['scores'][i];
      if (score != null) {
        // Saturate the score between 0.0 and 10.0 (displayed scores saturated
        // between 0 and 999).
        score = Math.min(Math.max(score, 0.0), 10.0);
      } else {
        score = null;
      }

      if (Array.from(systemLabel).includes('crash')) {
        activeCrashScores.push(score);
      } else {
        activeQualityScores.push(score);
        activeQualityScoresSeverity.push(severity);
        activeQualityWeightedScores.push(severity * score);
      }
    }
  });

  if (activeCrashScores.includes(null) || activeQualityScores.includes(null)) {
    return null;
  } else if (activeCrashScores.length == 0 && activeQualityScores.length == 0) {
    return 0.0;
  } else {
    if (d3.select('#aggregate')
            .select('input[name="aggregate"]:checked')
            .node().id == "hybrid_max_rss_function") {
      let aggregateCrashScore = Math.max.apply(Math, activeCrashScores);
      let aggregateQualityScore = null;

      if (activeQualityScores.length == 0) {
        // If no quality scores are selected, set to zero.
        aggregateQualityScore = 0.0;
      } else {
        let rss_severity = rootSumSquare(activeQualityScoresSeverity);
        if (rss_severity != 0.0) {
          aggregateQualityScore = rootSumSquare(activeQualityWeightedScores) /
          rss_severity;
        } else {
          // Only experimental scores (severity = 0) are selected.
          // - Return 0.0 if at least one crash score is selected.
          // - Else, return RSS(active_experimental_scores) / N
          if (activeCrashScores.length != 0) {
            aggregateQualityScore = 0.0;
          } else {
            aggregateQualityScore = rootSumSquare(activeQualityScores) /
            activeQualityScores.length;
          }
        }
      }

      return Math.max(aggregateCrashScore, aggregateQualityScore);
    } else {
      return Math.max.apply(
          Math, activeCrashScores.concat(activeQualityScores));
    }
  }
}

/**
 * Returns the color corresponding to a score.
 * @param {number} score The score value.
 * @param {Array<string>} colorMap The color map.
 * @return {string} Color code corresponding to the score.
 */
function colorForScore(score, colorMap) {
  if (score === null) {
    return '#888888';
  } else if (score === SIM_FAILURE) {
    return '#666666';
  } else {
    return colorMap(score);
  }
}

/**
 * Returns the text string corresponding to a score.
 * @param {number} score The score value.
 * @return {string} String corresponding to the score.
 */
function displayedScore(score) {
  if (score === null) {
    return 'NaN';
  } else if (score === SIM_FAILURE) {
    return 'Fail';
  } else {
    let displayed = Math.round(100 * score);
    return displayed < 999 ? displayed.toString() : '999';
  }
}

/**
 * Rescores all table cells. This should be called whenever the filters are
 * modified or the 'aggregate' function is updated.
 */
function rescore() {
  if (!d3.select('#manual_selection_button').property('checked')) {
    d3.select('#active_scores').selectAll('input').each(function(d, i) {
      if (d3.select(this).attr('active_flight_mode') == 'true' &&
          d3.select(this).attr('active_system') == 'true') {
        d3.select(this).property('checked', true);
      } else {
        d3.select(this).property('checked', false);
      }
    });
  }

  d3.select('#tables').selectAll('.cell').style('fill', function(d) {
    return colorForScore(aggregate(d), globalColorMap);
  });

  let listOfSelectedScores = [];
  d3.select('#tables').selectAll('.cell_text').text(function(d) {
    listOfSelectedScores.push(displayedScore(aggregate(d)));
    return displayedScore(aggregate(d));
  });

  updateScorePlot();
  renderStats(listOfSelectedScores);
}

/**
 * Render statistics for a list of selected scores and the complete batch.
 * @param {listOfSelectedScores} scores list filtered by metric configuration.
 */
function renderStats(listOfSelectedScores) {
  if (batchStatistics == null) {
    batchStatistics = getStats(listOfSelectedScores);
    totalSimsInBatch = listOfSelectedScores.length;
    [batchScoresHist] = getChartData(batchStatistics, 'Batch');
  }

  selectionStatistics = getStats(listOfSelectedScores);
  [selectionScoresHist, scoresLabels, selectionResultsHist, resultsLabels] =
      getChartData(selectionStatistics, 'Selection');

  let allScoresHist = [selectionScoresHist, batchScoresHist];
  let allResultsHist = [selectionResultsHist];

  let cumulativeScoreCutoff = d3.select('#input_cutoff').property('value');
  let cumulativeScore =
      getCumulativeScore(selectionScoresHist, cumulativeScoreCutoff);

  let resultschart = c3.generate({
    bindto: '#results_chart',
    padding: {top: 5, bottom: 20, right: 30},
    size: {height: 250, width: 250},
    data: {
      columns: allResultsHist,
      type: 'bar',
      labels: {format: {Selection: function(v) { return v + "%"; }}},
      colors: {
        Selection: 'rgb(130, 130, 130)',
      },
    },
    axis: {
      x: {
        label: {text: 'Results', position: 'outer-center'},
        type: 'category',
        categories: resultsLabels
      },
      y: {show: false}
    },
    tooltip: {
      format: {
        title: function(d) {
          if (d == 0) {
            return 'Simulation failures.'
          } else if (d == 1) {
            return 'Unable to score.'
          } else {
            return 'Scored simulations.'
          }
        }
      }
    },
    bar: {width: {ratio: 0.5}},
    legend: {show: false},
    title: {text: ['Total Simulations: ' + totalSimsInBatch]},
  });

  let scoreschart = c3.generate({
    bindto: '#scores_chart',
    padding: {
      top: 5,
      bottom: 20,
      left: 30,
    },
    size: {height: 250, width: 800},
    data: {
      columns: allScoresHist,
      type: 'bar',
      labels: {
        format: {
          Selection: function(v) { return v + "%"; },
          Batch: function(v) { return v + "%"; }
        }
      },
      colors: {Selection: 'rgb(130, 130, 130)', Batch: 'black'}
    },
    axis: {
      x: {
        label: {text: 'Scores', position: 'outer-center'},
        type: 'category',
        categories: scoresLabels
      },
      y: {
        show: false,
        max: 100
      }
    },
    bar: {width: {ratio: 0.75}},
    legend: {position: 'right'},
    title: {text: 'Distribution of Scored Simulations.'},
    regions: [
      {axis: 'x', opacity: 1.0, start: -0.5, end: 0.5, class: 'region_0_20'},
      {axis: 'x', opacity: 1.0, start: 0.5, end: 1.5, class: 'region_20_40'},
      {axis: 'x', opacity: 1.0, start: 1.5, end: 2.5, class: 'region_40_60'},
      {axis: 'x', opacity: 1.0, start: 2.5, end: 3.5, class: 'region_60_80'},
      {axis: 'x', opacity: 1.0, start: 3.5, end: 4.5, class: 'region_80_100'},
      {axis: 'x', opacity: 0.9, start: 4.5, end: 5.5, class: 'region_100_300'},
      {axis: 'x', opacity: 0.95, start: 5.5, end: 6.5, class: 'region_300_600'},
      {axis: 'x', opacity: 1.0, start: 6.5, end: 7.5, class: 'region_600_999'},
    ]
  });

  let batchscore = c3.generate({
    bindto: '#batch_score',
    padding: {top: 5, bottom: 20, right: 30},
    size: {height: 250, width: 200},
    data: {
      columns: [['Score', cumulativeScore]],
      type: 'gauge',
      colors: {
        'Score': 'rgb(130, 130, 130)',
      },
    },
    gauge: {
      fullCircle: true,
      width: 30,
      label: {format: function(value, ratio) { return value + ' %'; }},
    },
    title: {text: 'Sims with score less than:'},
    legend: {show: false}
  });

  // Update score display on user selection of cutoff.
  d3.select('#input_cutoff')
      .on('change',
          function() {
            let cumulativeScore =
                getCumulativeScore(selectionScoresHist, this.value);
            batchscore.load({columns: [['Score', cumulativeScore]]});
          })

          function getCumulativeScore(selectionScoresHist, cutOff) {
            let cumulativeScore = 0;
            let maxIdx = Math.ceil(cutOff / 20);
            maxIdx = Math.min(maxIdx, selectionScoresHist.length);
            for (let i = 1; i <= maxIdx; i++) {
              cumulativeScore += selectionScoresHist[i];
            }
            return cumulativeScore;
          }

  function getStats(listOfScores) {
    let totalSims = listOfScores.length;
    let counts = countOccurance(listOfScores);
    let totalScored = totalSims - counts["NaN"] - counts["Fail"];
    counts["Scored"] = totalScored;
    let divisor = [];
    let stats = [];
    for (x in counts) {
      if (x == "Scored" || x == "NaN" || x == "Fail")
        divisor = totalSims;
      else
        divisor = totalScored;

      stats[x] = Math.round(100.0 * counts[x] / divisor);
    }
    return stats;
  }

  function countOccurance(listOfScores) {
    let counts = [];
    // This initialization order will also be the print order in the table.
    counts["Fail"] = 0;
    counts["NaN"] = 0;
    counts["Scored"] = 0;
    counts["0 to 20"] = 0;
    counts["20 to 40"] = 0;
    counts["40 to 60"] = 0;
    counts["60 to 80"] = 0;
    counts["80 to 100"] = 0;
    counts["100 to 300"] = 0;
    counts["300 to 600"] = 0;
    counts["600 to 999"] = 0;

    for (let i = 0; i < listOfScores.length; ++i) {
      let score = listOfScores[i];
      if (score == "Fail") {
        counts["Fail"]++;
      } else if (score == "NaN") {
        counts["NaN"]++;
      } else if (Number(score) >= 600) {
        counts["600 to 999"]++;
      } else if (Number(score) >= 300 && Number(score) < 600) {
        counts["300 to 600"]++;
      } else if (Number(score) >= 100 && Number(score) < 300) {
        counts["100 to 300"]++;
      } else if (Number(score) >= 80 && Number(score) < 100) {
        counts["80 to 100"]++;
      } else if (Number(score) >= 60 && Number(score) < 80) {
        counts["60 to 80"]++;
      } else if (Number(score) >= 40 && Number(score) < 60) {
        counts["40 to 60"]++;
      } else if (Number(score) >= 20 && Number(score) < 40) {
        counts["20 to 40"]++;
      } else if (Number(score) >= 0 && Number(score) < 20) {
        counts["0 to 20"]++;
      }
    }

    return counts;
  }

  function getChartData(stats, legend) {
    let scoresHist = [legend];
    let scoresLabels = [];
    let resultsHist = [legend];
    let resultsLabels = [];
    for (x in stats) {
      if (x != "Scored" && x != "NaN" && x != "Fail") {
        scoresHist.push(stats[x]);
        scoresLabels.push(x);
      } else {
        resultsHist.push(stats[x]);
        resultsLabels.push(x);
      }
    }
    return [scoresHist, scoresLabels, resultsHist, resultsLabels];
  }

  d3.select(".c3-region.region_0_20")
      .style('fill', globalColorMap(0 / 100));
  d3.select(".c3-region.region_20_40").style('fill', globalColorMap(20 / 100));
  d3.select(".c3-region.region_40_60").style('fill', globalColorMap(40 / 100));
  d3.select(".c3-region.region_60_80").style('fill', globalColorMap(60 / 100));
  d3.select(".c3-region.region_80_100").style('fill', globalColorMap(80 / 100));
  d3.select(".c3-region.region_100_300")
      .style('fill', globalColorMap(100 / 100));
  d3.select(".c3-region.region_300_600")
      .style('fill', globalColorMap(100 / 100));
  d3.select(".c3-region.region_600_999")
      .style('fill', globalColorMap(100 / 100));
}

/**
 * Draws a score table.
 * @param {string} elt ID of the containing element. The new table will be
 *     appended to it.
 * @param {Array<dict>} metrics List of all metrics used for scoring.
 * @param {Object} tableSpec Specifier for the table, generated by
 *     makeTableSpec.
 * @param {Array<string>} colorMap: The color map applied to scores.
 */
function drawTable(elt, metrics, tableSpec, colorMap) {
  function jobSummaryText(d) {
    let parts = ['Job ID: ' + d['job_id']];
    if (!d['sim_success']) {
      parts.push('Simulator error');
    } else {
      let activeScoreIndices = [];
      d3.select('#active_scores').selectAll('input').each(function(d, i) {
        if (d3.select(this).property('checked')) {
          activeScoreIndices.push(i);
        }
      });
      // assumption that metrics and d['scores'][i] are aligned
      if (activeScoreIndices.length > 0) {
        for (let i = 0; i < metrics.length; i++) {
          if (activeScoreIndices.includes(i)) {
            let score = d['scores'][i];
            if (score != null) {
              score = Math.max(0.0, d['scores'][i]);
            } else {
              score = null;
            }
            parts.push(`<font color = '${colorForScore(score, colorMap)}'> ${
                    metrics[i]['name']} (S-${metrics[i]['severity']})
                    : ${displayedScore(score)}</ font>`);
          }
        }
      } else {
        parts.push('<font color =red> NO SCORES SELECTED </ font>');
      }
    }
    return parts.join("<br>");
  }

  let scoreTable = d3.select(elt)
                       .append('svg')
                       .attr('width', tableSpec.width)
                       .attr('height', tableSpec.height)
                       .attr('class', 'score_table');
  let xlabel = tableSpec.data['xlabel'];
  let ylabel = tableSpec.data['ylabel'];
  let title = tableSpec.data['title'];

  // Title.
  scoreTable.append('text')
      .attr('x', tableSpec.leftPadding + tableSpec.bodyWidth / 2)
      .attr('y', 5)
      .attr('dy', '1.1em')
      .attr('class', 'title_text')
      .style('text-anchor', 'middle')
      .text(title)
      .call(wrapText, tableSpec.bodyWidth);

  // x-axis label.
  if (xlabel !== 'Row') {
    scoreTable.append('text')
        .attr('x', tableSpec.leftPadding + tableSpec.bodyWidth / 2)
        .attr(
            'y', tableSpec.topPadding + tableSpec.bodyHeight +
                tableSpec.bottomPadding * 0.75)
        .style('text-anchor', 'middle')
        .text(tableSpec.data['xlabel']);
  }

  // y-axis label.
  if (ylabel !== 'Column' && ylabel !== title) {
    let x = tableSpec.leftPadding * 0.25;
    let y = tableSpec.topPadding + tableSpec.bodyHeight / 2;
    scoreTable.append('text')
        .attr('x', x)
        .attr('y', y)
        .attr('dominant-baseline', 'central')
        .attr('text-anchor', 'middle')
        .attr('transform', `rotate(-90, ${x}, ${y})`)
        .text(tableSpec.data['ylabel']);
  }

  // Table cells.
  scoreTable.selectAll('a')
      .data(tableSpec.data['job_data'])
      .enter()
      .append('a')
      .attr(
          'xlink:href', function(d) { return `overrides_${d['job_id']}.html`; })
      .append('svg:rect')
      .attr('class', 'cell')
      .attr(
          'x',
          function(d, i) {
            return tableSpec.leftPadding + d['table_pos'][0] * TABLE_CELL_WIDTH;
          })
      .attr(
          'y',
          function(d, i) {
            return (
                tableSpec.topPadding + tableSpec.bodyHeight -
                (d['table_pos'][1] + 1) * TABLE_CELL_HEIGHT);
          })
      .attr('width', TABLE_CELL_WIDTH)
      .attr('height', TABLE_CELL_HEIGHT)
      .style(
          'fill', function(d) { return colorForScore(aggregate(d), colorMap); })
      .style('stroke', '#555')
      .style('stroke-width', '1px')
      .style('border', '1px solid #DDDDDD')
      .style('padding', '1px')
      .on('mouseover',
          function(d) {
            d3.select(this)
                .style('stroke', 'rgb(0,0,0)')
                .style('stroke-width', '2.5px')
                .style('border', '1px solid #FFFFFF')
                .style('padding', '2px');
            d3.select('#job_summary')
                .style('opacity', 0.8)
                .html(jobSummaryText(d));
          })
      .on('mouseout', function(d) {
        d3.select(this)
            .style('stroke', '#555')
            .style('stroke-width', '1px')
            .style('border', '1px solid #DDDDDD')
            .style('padding', '1px');
        d3.select('#job_summary').style('opacity', 0).html('');
      });

  // Cell labels.
  scoreTable.selectAll('.label')
      .data(tableSpec.data['job_data'])
      .enter()
      .append('svg:text')
      .attr('class', 'cell_text')
      .attr(
          'x',
          function(d, i) {
            return tableSpec.leftPadding +
                (d['table_pos'][0] + 0.5) * TABLE_CELL_WIDTH;
          })
      .attr(
          'y',
          function(d, i) {
            return (
                tableSpec.topPadding + tableSpec.bodyHeight -
                (d['table_pos'][1] + 0.5) * TABLE_CELL_HEIGHT);
          })
      .attr('text-anchor', 'middle')
      .attr('dy', '.35em')
      .attr('pointer-events', 'none')  // Pass mouse events down to cell.
      .text(function(d) { return displayedScore(aggregate(d)); });

  // x-axis tick labels.
  scoreTable.selectAll('.xtick')
      .data(tableSpec.data['xticks'])
      .enter()
      .append('svg:text')
      .attr(
          'x',
          function(d, i) {
            return tableSpec.leftPadding + (i + 0.5) * TABLE_CELL_WIDTH;
          })
      .attr(
          'y', (tableSpec.topPadding + tableSpec.bodyHeight +
                0.25 * tableSpec.bottomPadding))
      .attr('text-anchor', 'middle')
      .attr('dy', '.35em')
      .text(Object);

  // y-axis tick labels.
  scoreTable.selectAll('.xtick')
      .data(tableSpec.data['yticks'])
      .enter()
      .append('svg:text')
      .attr('x', 0.9 * tableSpec.leftPadding)
      .attr(
          'y',
          function(d, i) {
            return (
                tableSpec.topPadding + tableSpec.bodyHeight -
                (i + 0.5) * TABLE_CELL_HEIGHT);
          })
      .attr('text-anchor', 'end')
      .attr('dy', '.35em')
      .text(Object);
}

function isWindSweep(data) {
  return getWindSpeedAndShear(data['table_data'][0]) != false;
}

function getWindSpeedAndShear(table) {
  // TODO: Include the wind speeds and shears in overview_data.json.
  title = table['title'];
  if (!title.startsWith('Monte Carlo Wind Speed')) {
    return false;
  }
  parts = title.split('=');
  windSpeed = +(parts[1].split(',')[0].trim());
  windShear = +(parts[2].trim());
  return [windSpeed, windShear];
}

// Groups aggregated scores by wind speed and wind shear, and returns a dict
// containing the speed, shear, and the list of scores.
function preparePlotData(data) {
  function getAggregateScores(table) {
    jobs = table['job_data'];
    scores = [];
    for (let i = 0; i < jobs.length; i++) {
      score = aggregate(jobs[i]);
      if (score === null || score === SIM_FAILURE) {
        score = 10.0;
      }
      scores.push(100 * score);
    }
    return scores;
  }

  scoreData = []
  for (let i=0; i<data['table_data'].length; i++) {
    table = data['table_data'][i];
    [windSpeed, windShear] = getWindSpeedAndShear(table);
    scoreData.push({'windSpeed': windSpeed,
                     'windShear': windShear,
                     'scores': getAggregateScores(table).sort(
                         function(a, b){return a - b})});
  }
  return scoreData;
}

// Computes power for each run and groups by wind speed and wind shear,
// and returns a dict containing the speed, shear, and the list of scores.
function preparePowerPlotData(data) {
  function getPowers(table, index) {
    jobs = table['job_data'];
    scores = []
    for (let i = 0; i < jobs.length; i++) {
      score = jobs[i].scores[index];
      if (!jobs[i]['sim_success'] || score === null || score === SIM_FAILURE) {
        score = -600.0;
      } else {
        score = Math.max(-600.0, 600.0 * (1.0 - score));
      }
      scores.push(score);
    }
    return scores;
  }
  // Locate index of power score.
  index = -1;
  for (let i = 0; i < data.metrics.length; i++) {
    if (data.metrics[i].system_labels.includes('performance')) {
      index = i;
      break;
    }
  }
  powerData = []
  if (index >= 0) {
    for (let i=0; i<data['table_data'].length; i++) {
      table = data['table_data'][i];
      [windSpeed, windShear] = getWindSpeedAndShear(table);
      powerData.push({'windSpeed': windSpeed,
                      'windShear': windShear,
                      'scores': getPowers(table, index).sort(
                          function(a, b){return a - b})});
    }
  }
  return powerData;
}

function updateScorePlot() {
  data = preparePlotData(scorePlotData.runData);
  updateQuantilePlot(scorePlotData.plot, data);
}

/**
 * Builds a quantile plot.
 * @param {Array<float>} domain Plot domain.
 * @param {Array<float>} range Plot range.
 * @param {string} title Title of the plot.
 * @param {string} yLabel Label of the y-axis.
 * @param {boolean} linearScale Use a linear scale on the y axis (instead of an
 *     exponential scale).
 * @param {boolean} reverse Reverse the quantile order so a higher score is
 *     better instead of worse.
 */
function buildQuantilePlot(domain, range, title, yLabel, linearScale, reverse) {
  var plot = {};

  var svgWidth = 1000;
  var svgHeight = 600;
  var margin = {top: 30, right: 20, bottom: 60, left: 50};
  var width = svgWidth - margin.left - margin.right;
  var height = svgHeight - margin.top - margin.bottom;
  var quantile_lines = [{'q': 0.0, 'color': 'steelblue', 'width': 1.3},
                        {'q': 0.1, 'color': 'black', 'width': 0.5},
                        {'q': 0.25, 'color': 'black', 'width': 0.5},
                        {'q': 0.5, 'color': 'black', 'width': 2},
                        {'q': 0.75, 'color': 'black', 'width': 0.5},
                        {'q': 0.9, 'color': 'black', 'width': 0.5},
                        {'q': 1.0, 'color': 'red', 'width': 1.0}];
  var quantile_bands = [{'q0': 0.1, 'q1': 0.9, 'color': '#e0e5ff'},
                        {'q0': 0.25, 'q1': 0.75, 'color': '#c0cdff'}];

  var svg = d3.select('#plots').append('svg')
      .attr('width', svgWidth).attr('height', svgHeight);
  var g = svg.append('g')
      .attr('transform', 'translate(' + margin.left + ',' + margin.top + ')');
  var x = d3.scaleLinear().rangeRound([0, width]);
  if (linearScale) {
    var y = d3.scaleLinear().rangeRound([0, height]);
  } else {
    var y = d3.scalePow().exponent(0.33).rangeRound([0, height]);
  }
  x.domain(domain);
  y.domain(range);
  g.append('g')
      .attr('transform', 'translate(0,' + height + ')')
      .call(d3.axisBottom(x))
      .select('.domain');
  g.append('text')
      .attr('transform', 'translate(' + (width / 2) + ' ,'
            + (height + margin.top + 20) + ')')
      .style('text-anchor', 'middle')
      .text('Wind speed [m/s]');
  g.append('g')
      .call(d3.axisLeft(y))
      .select('.domain');
  g.append('text')
      .attr('transform', 'rotate(-90)')
      .attr('y', 0 - margin.left)
      .attr('x', 0 - (height / 2))
      .attr('dy', '1em')
      .style('text-anchor', 'middle')
      .text(yLabel);
  g.append('text')
      .attr('x', width / 2)
      .attr('y', -(margin.top / 2))
      .style('text-anchor', 'middle')
      .style('font-size', '20px')
      .text(title)
  plot.areas = [];
  for (let i = 0; i < quantile_bands.length; i++) {
    let q0 = reverse ? (1.0 - quantile_bands[i].q0) : quantile_bands[i].q0;
    let q1 = reverse ? (1.0 - quantile_bands[i].q1) : quantile_bands[i].q1;
    let area = d3.area().x(function(d) { return x(d.windSpeed); })
        .y0(function(d) { return y(d3.quantile(d.scores, q0)); })
        .y1(function(d) { return y(d3.quantile(d.scores, q1)); })
        .defined(function(d) { return d.windShear == minWindShear; });
    area.styling = quantile_bands[i];
    plot.areas.push(area)
  }
  plot.lines = [];
  for (let i = 0; i < quantile_lines.length; i++) {
    let q = reverse ? (1.0 - quantile_lines[i].q) : quantile_lines[i].q;
    let line = d3.line().x(function(d) { return x(d.windSpeed); })
        .y(function(d) { return y(d3.quantile(d.scores, q)); })
        .defined(function(d) { return d.windShear == minWindShear; });
    line.styling = quantile_lines[i];
    plot.lines.push(line);
  }
  plot.g = g;
  return plot;
}

function updateQuantilePlot(plot, data) {
  plot.g.selectAll('.plotline').remove();
  for (let i = 0; i < plot.areas.length; i++) {
    let area = plot.areas[i];
    plot.g.append('path').datum(data)
        .attr('fill', area.styling.color)
        .attr('stroke','none')
        .attr('class', 'plotline')
        .attr('d', area);
  }
  for (let i = 0; i < plot.lines.length; i++) {
    let line = plot.lines[i];
    plot.g.append('path').datum(data)
        .attr('fill', 'none')
        .attr('stroke', line.styling.color)
        .attr('stroke-linejoin', 'round')
        .attr('stroke-linecap', 'round')
        .attr('stroke-width', line.styling.width)
        .attr('class', 'plotline')
        .attr('d', line);
  }
}

function drawPlots(data) {
  if (!isWindSweep(data)) {
    return false;
  }

  // Build and render power plot.
  powerData = preparePowerPlotData(data);
  minWindShear = d3.min(powerData, function(d) { return d.windShear; });
  minWindSpeed = d3.min(powerData, function(d) { return d.windSpeed; });
  maxWindSpeed = d3.max(powerData, function(d) { return d.windSpeed; });
    [-600, 800]

  powerPlot = buildQuantilePlot(
      [minWindSpeed, maxWindSpeed], [800, -600],
      'Generated power distribution (for wind shear = ' + minWindShear + ')',
      'Mean generated power [kW]', true, true);
  updateQuantilePlot(powerPlot, powerData);

  // Build score plot.
  scorePlot = buildQuantilePlot(
      [minWindSpeed, maxWindSpeed], [999, 0],
      'Aggregate score distribution (for wind shear = ' + minWindShear + ')',
      'Score', false, false)

  scorePlotData.runData = data;
  scorePlotData.plot = scorePlot;
}

/**
 * Makes data fully specifying a score table.
 * @param {dict} tableData Element from the 'table_data' list in the JSON
 *     overview data generated by a sweeps batch sim.
 * @return {Object} Object specifying a table element.
 */
function makeTableSpec(tableData) {
  let bodyWidth = tableData['num_cols'] * TABLE_CELL_WIDTH;
  let bodyHeight = tableData['num_rows'] * TABLE_CELL_HEIGHT;
  let leftPadding = 1.5 * TABLE_CELL_WIDTH;
  let topPadding = TABLE_CELL_HEIGHT;
  let bottomPadding = TABLE_CELL_HEIGHT;

  return {
    bodyWidth: bodyWidth,
    bodyHeight: bodyHeight,
    leftPadding: leftPadding,
    topPadding: topPadding,
    bottomPadding: bottomPadding,
    width: leftPadding + bodyWidth,
    height: bottomPadding + bodyHeight + topPadding,
    data: tableData,
  };
}

/**
 * Renders the score tables on a sweeps report page.
 * @param {dict} data The JSON overview data generated by a sweeps batch sim.
 * @param {string} elt ID of the HTML element that will contain the tables.
 */
function renderScoreTables(data, elt) {
  document.title = data['title'];
  d3.select('#report_title').text(data['title']);
  d3.select('#report_metadata').html(function() {
    parts = ['Commit: ' + data['commit']];
    if (data['parameter_seed']) {
      parts.push('Parameter seed: ' + data['parameter_seed']);
    }
    return parts.join('<br>');
  });

  makeCheckboxes('#active_scores', data['metrics']);

  let colorMap = d3.scaleQuantize().domain([0, 1.2]).range(data['color_map']);
  globalColorMap = colorMap;
  drawLegend(elt, colorMap);

  let numMajorRows =
      Math.ceil(data['table_data'].length / data['num_major_cols']);
  let majorRowNumbers = [];
  for (let i = 0; i < numMajorRows; i++) {
    majorRowNumbers.push(i);
  }
  d3.select('#tables')
      .selectAll('.major_row')
      .data(majorRowNumbers)
      .enter()
      .append('div')
      .attr('class', 'major_row')
      .attr('id', function(d) { return 'row' + d; });

  for (let i = 0; i < data['table_data'].length; i++) {
    let majorRowIndex = Math.floor(i / data['num_major_cols']);
    let tableSpec = makeTableSpec(data['table_data'][i]);
    drawTable('#row' + majorRowIndex, data['metrics'], tableSpec, colorMap);
  }
  drawPlots(data);
  rescore();
}

/**
 * Renders the score tables on a sweeps report page.
 * @param {string} filename Path of JSON file containing overview data generated
 *     by a sweeps batch sim.
 */
function renderScoreTablesFromFile(filename) {
  d3.json(filename, function(error, contents) {
    if (error) {
      console.log('Problem loading file ' + filename);
      console.log(error);
      d3.select('#main_section')
          .append('div')
          .style('display', 'inline-block')
          .style('text-align', 'left')
          // clang-format off
         .html(`This page must be viewed from a web server because it loads
local files. If attempting to view in your browser with a file:// link, instead:
<ol>
<li>Open a terminal in the directory containing this file.
<li>Run "<tt>python -m SimpleHTTPServer 1234</tt>". You can replace
    <tt>1234</tt> with your favorit port number.
<li>Navigate to <a href="http://localhost:1234">localhost:1234</a> in your
    browser.
</ol>`);
      // clang-format on
      return;
    }
    renderScoreTables(contents, 'body');
  });
}
