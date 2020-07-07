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
 * @constructor A class to plot strip charts using a list of values.
 *
 * A strip chart plots data series as lines, and moves the lines
 * toward the left when new data points arrive.
 *
 * @param {hashmap} properties A map of parameters.
 */
function StripChartFromList(properties) {
  properties.x_attrs.invert = true;
  this.histories = [];

  // A list of data series.
  this.xHistory = [];
  // A list of plotted lines.
  this.paths = [];

  BasePlot.call(this, properties);
  var xLimits = this.simpleGraph.getXLimits();
  this.bound = xLimits[1] - xLimits[0];

  if ('keys' in properties) {
    this.keys = properties.keys;
    var colors = DEFAULT_COLORS;
    for (var i = 0; i < this.keys.length; i++) {
      var color = colors[i];
      var key = this.keys[i];
      var coords = [];
      this.histories.push(coords);
      var path = this.plotLine(coords, color, 1, 'series', null, key, i);
      this.paths.push(path);
    }
  }
}


/** Inherit BasePlot. */
StripChartFromList.prototype = Object.create(BasePlot.prototype);


/** Set the constructor. */
StripChartFromList.prototype.constructor = StripChartFromList;


/**
 * Update the plot when new data arrives from periodic AJAX polls.
 *
 * @param {hashmap} resp Response from the periodic data poll.
 * @param {bool} resetUponUpdate Reset the graph when new data is received.
 */
StripChartFromList.prototype.update = function(resp, resetUponUpdate) {

  // resp is an object with new plotting data.
  var values = getYCoordListWithOneXCoord(
      resp, this.xSrc, this.ySrc, this.xWrap, this.yWrap);
  if (isBadType(values)) {
    return;
  }
  var yCoords = values.y;
  var xCoord = values.x;

  if (isBadNumber(xCoord)) {
    return;
  }

  var yMin = Number.MAX_VALUE;
  var yMax = -Number.MAX_VALUE;
  for (var i = 0; i < this.histories.length; i++) {
    var coords = this.histories[i];
    var newBound = updateBound(coords, yMin, yMax, 'y');
    yMin = newBound.min;
    yMax = newBound.max;
  }
  // Auto-update y axis domain.
  this.autoUpdateYDomain(
      yCoords.filter(function(i) {return !isBadType(i)}), yMin, yMax);

  var colors = DEFAULT_COLORS;
  for (var i = 0; i < yCoords.length; i++) {
    // Plot line segment.
    if (this.histories.length <= i) {
      var color = colors[i];
      var key = i.toString();
      if (this.keys) {
        key = this.keys[i];
      }
      var coords = [];
      this.histories.push(coords);
      var path = this.plotLine(coords, color, 1, 'series', null, key, i);
      this.paths.push(path);
    }
  }
  if (resetUponUpdate) {
    this.simpleGraph.updateScale();
  }
  this.refresh(xCoord, yCoords);

  this.drawLegend(resetUponUpdate);
};


/**
 * Update existing lines.
 *
 * @param {float} xCoord Value of the x coordinate shared by all y coordinates.
 * @param {array} yCoords An array of y coordinates.
 */
StripChartFromList.prototype.refresh = function(xCoord, yCoords) {
  if (this.histories.length) {
    if (isBadNumber(xCoord)) {
      return;
    }
    this.xHistory.push(xCoord);
    var xScale = this.x;
    var yScale = this.y;
    var upperBound = this.xHistory[this.xHistory.length - 1];
    var lowerBound = upperBound - this.bound;
    var lineCallback = d3.svg.line()
        .x(function(d) { return xScale(upperBound - d.x); })
        .y(function(d) { return yScale(d.y); });
    while (this.xHistory[this.xHistory.length - 1] - this.xHistory[0] >
           this.bound) {
      this.xHistory.shift();
    }
    for (var i = 0; i < this.histories.length; i++) {
      var path = this.paths[i];
      var coords = this.histories[i];
      if (yCoords && !isBadNumber(yCoords[i])) {
        coords.push({'x': xCoord, 'y': yCoords[i]});
      }
      while (coords.length > 0 && coords[0].x < lowerBound) {
        coords.shift();
      }

      path.attr('d', lineCallback);
    }
  }
}
