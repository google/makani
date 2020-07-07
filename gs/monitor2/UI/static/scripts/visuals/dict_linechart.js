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

/**
 * @constructor Class to plot lines that get updated with a hashmap of values.
 *
 * @param {hashmap} properties A map of parameters.
 */

function LineChartFromDict(properties) {
  if ('keys' in properties) {
    this.keys = properties.keys;
  }
  this.seriesSrc = properties.series_src;
  this.useMarkers = properties.use_markers;
  // A list of plotted lines.
  this.paths = [];

  BasePlot.call(this, properties);
}


/** Inherit BasePlot. */
LineChartFromDict.prototype = Object.create(BasePlot.prototype);


/** Set the constructor. */
LineChartFromDict.prototype.constructor = LineChartFromDict;


/**
 * Update the plot when new data arrives from periodic AJAX polls.
 *
 * @param {hashmap} resp Response from the periodic data poll.
 * @param {bool} resetUponUpdate Reset the graph when new data is received.
 */
LineChartFromDict.prototype.update = function(resp, resetUponUpdate) {
  var series = getMapOfCoordLists(resp, this.seriesSrc, this.xWrap,
                                  this.yWrap);
  if (series == null) {
    return;
  }
  var keys = self.keys;
  if (!keys) {
    keys = Object.keys(series);
    keys.sort();
  }

  var xLimits = [Number.MAX_VALUE, -Number.MAX_VALUE];
  // SimpleGraph uses inverted y-axis, so yLimits should be computed as
  // [max, min].
  var yLimits = [Number.MAX_VALUE, -Number.MAX_VALUE];
  for (var key in series) {
    if (series.hasOwnProperty(key)) {
      var bounds = d3.extent(series[key]['x']);
      if (bounds[0] < xLimits[0]) {
        xLimits[0] = bounds[0];
      }
      if (bounds[1] > xLimits[1]) {
        xLimits[1] = bounds[1];
      }
      bounds = d3.extent(series[key]['y']);
      if (bounds[0] < yLimits[0]) {
        yLimits[0] = bounds[0];
      }
      if (bounds[1] > yLimits[1]) {
        yLimits[1] = bounds[1];
      }
    }
  }
  this.simpleGraph.setXDomain(xLimits);
  this.simpleGraph.setYDomain(yLimits);
  if (resetUponUpdate) {
    this.simpleGraph.updateScale();
  }
  this.simpleGraph.redraw()();

  var obj = this;
  var lineCallback = d3.svg.line()
      .x(function(d) {return obj.x(d.x); })
      .y(function(d) {return obj.y(d.y); });
  this.markerXCallback = function(d) {return obj.x(d.x);};
  this.markerYCallback = function(d) {return obj.y(d.y);};
  var markerRadius = 1;
  this.lineCallback = lineCallback;

  var colors = DEFAULT_COLORS;
  for (var p in this.paths) {
    this.paths[p].remove();
  }
  this.paths = [];
  for (var i = 0; i < keys.length; i++) {
    var key = keys[i];
    var xCoords = series[key]['x'];
    var yCoords = series[key]['y'];
    var color = colors[i];

    var coords = [];
    for (var c = 0; c < xCoords.length; c++) {
      coords.push({'x': xCoords[c], 'y': yCoords[c]});
    }

    var path;
    if (this.useMarkers) {
      path = this.plotMarkers(
          coords, color, this.markerXCallback, this.markerYCallback,
          markerRadius, key, i);
    } else {
      path = this.plotLine(coords, color, 1, 'series', this.lineCallback,
                           key, i);
    }
    this.paths.push(path);
  }

  this.drawLegend(resetUponUpdate);
};


/** Update existing lines. */
LineChartFromDict.prototype.refresh = function() {
  if (this.paths) {
    for (var i = 0; i < this.paths.length; i++) {
      if (this.useMarkers) {
        this.paths[i]
            .attr('cx', this.markerXCallback)
            .attr('cy', this.markerYCallback);
      } else {
        this.paths[i].attr('d', this.lineCallback);
      }
    }
  }
}
