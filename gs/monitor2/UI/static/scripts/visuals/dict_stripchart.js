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
 * @constructor A class to plot strip charts using a dict of values.
 *
 * A strip chart plots data series as lines, and moves the lines
 * toward the left when new data points arrive.
 *
 * @param {hashmap} properties A map of parameters.
 */
function StripChartFromDict(properties) {
  properties.x_attrs.invert = true;
  // A map of data series, indexed by keys.
  this.histories = {};
  // A map of plotted lines, indexed by keys.
  this.paths = {};

  BasePlot.call(this, properties);
  var xLimits = this.simpleGraph.getXLimits();
  this.bound = xLimits[1] - xLimits[0];

  // Keys are used to distinguish lines and draw legends.
  if ('keys' in properties) {
    this.keys = properties.keys;
    var colors = DEFAULT_COLORS;
    for (var i = 0; i < this.keys.length; i++) {
      var key = this.keys[i];
      // Create a new line when new data is observed.
      var color = colors[i];
      var coords = [];
      this.histories[key] = coords;
      var path = this.plotLine(coords, color, 1, 'series', null, key, i);
      this.paths[key] = path;
    }
  } else {
    this.keys = null;
  }

  this.xBase = null;
}


/** Inherit BasePlot. */
StripChartFromDict.prototype = Object.create(BasePlot.prototype);


/** Set the constructor. */
StripChartFromDict.prototype.constructor = StripChartFromDict;


/**
 * Update the plot when new data arrives from periodic AJAX polls.
 *
 * @param {hashmap} resp Response from the periodic data poll.
 * @param {bool} resetUponUpdate Reset the graph when new data is received.
 */
StripChartFromDict.prototype.update = function(resp, resetUponUpdate) {

  // resp is an object with new plotting data.
  var values = getCoordMap(resp, this.ySrc, this.yWrap);
  if (isBadType(values)) {
    return;
  }
  var availableKeys = values.keys;
  if (this.keys) {
    var keys = this.keys;
    availableKeys = availableKeys.filter(
        function(key) {return keys.indexOf(key) != -1});
  }
  var yCoords = values.values;

  var values = getCoordMap(resp, this.xSrc, this.xWrap);
  if (isBadType(values)) {
    return;
  }
  var xCoords = values.values;

  var yValues = availableKeys
      .filter(function(key) {return !isBadType(yCoords[key])})
      .map(function(key) {return yCoords[key];});

  var yMin = Number.MAX_VALUE;
  var yMax = -Number.MAX_VALUE;
  var keys = Object.keys(this.histories);
  for (var i = 0; i < keys.length; i++) {
    var key = keys[i];
    var coords = this.histories[key];
    var newBound = updateBound(coords, yMin, yMax, 'y');
    yMin = newBound.min;
    yMax = newBound.max;
  }
  this.autoUpdateYDomain(yValues, yMin, yMax);

  var keys;
  if (!this.keys) {
    keys = availableKeys;
    keys.sort();
  } else {
    keys = this.keys;
  }

  var colors = DEFAULT_COLORS;
  for (i = 0; i < keys.length; i++) {
    var key = keys[i];
    if (key in yCoords && (!isBadType(yCoords[key])) &&
        key in xCoords && !(key in this.histories)) {
      // Create a new line when new data is observed.
      var color = colors[i];
      var coords = [];
      this.histories[key] = coords;
      var path = this.plotLine(coords, color, 1, 'series', null, key, i);
      this.paths[key] = path;
    }
  }
  if (resetUponUpdate) {
    this.simpleGraph.updateScale();
  }
  this.refresh(xCoords, yCoords);

  this.drawLegend(resetUponUpdate);
};


/**
 * Update existing lines.
 *
 * @param {hashmap} xCoords Values of x coordinates indexed by keys.
 * @param {hashmap} yCoords Values of y coordinates indexed by the same set of
 *     keys.
 */
StripChartFromDict.prototype.refresh = function(xCoords, yCoords) {
  if (!(xCoords && yCoords))
    return;
  var keys = Object.keys(this.histories);
  var xScale = this.x;
  var yScale = this.y;
  var xValues = keys.map(function(key) {return xCoords[key];})
                    .filter(function(x) {return !isBadType(x)});
  var upperBound = Math.max.apply(null, xValues);
  if (isNaN(upperBound))
    return;
  var lowerBound = upperBound - this.bound;
  var lineCallback = d3.svg.line()
      .x(function(d) { return xScale(upperBound - d.x); })
      .y(function(d) { return yScale(d.y); });

  for (var i = 0; i < keys.length; i++) {
    var key = keys[i];
    var path = this.paths[key];
    var coords = this.histories[key];
    if (xCoords && key in xCoords && !(isBadType(xCoords[key])) &&
        yCoords && key in yCoords && !(isBadType(yCoords[key]))) {
      coords.push({'x': xCoords[key], 'y': yCoords[key]});
    }
    while (coords.length > 0 && coords[0].x < lowerBound) {
      coords.shift();
    }

    path.attr('d', lineCallback);
  }
}
