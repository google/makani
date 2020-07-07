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
 * @param {hashmap} properties A map of parameters. It contains:
 *     {string} seriesSrcs Index to the data series in the response.
 *         It should point to a structure like:
 *         E.g. {'series_A': {'x': 0, y': 1}, 'series_B': {'x': 3.0, 'y': 2.0'}}
 *     {string} xlabel The X-axis label.
 *     {string} ylabel The Y-axis label.
 *     {list} xlim The lower and upper bounds for X-axis.
 *     {list} ylim The lower and upper bounds for Y-axis.
 *     {list} polygons Polygons to draw in the background, each is a hashmap:
 *         {'color': 'blue', 'x': [1.1, 1.2, ...], 'y': [3.2, 4.5, ...]}
 *     {integer} history_len Length of the history to keep.
 *     {bool} show_legend True if line labels should be drawn.
 *     {hashmap} line_properties Properties of lines keyed by series name.
 *         Each line property is a hashmap including 'color' and 'style'.
 *     {hashmap} marker_properties Properties of markers keyed by series name.
 *         Each marker property is a hashmap including 'color' and 'size'.
 */

function Sketch2DWindow(properties) {
  this.seriesSrcs = properties.series_srcs;
  this.dynamicsSrcs = properties.dynamic_polygons;
  this.markersSrcs = properties.markers;
  this.polygons = properties.polygons;
  this.historyLen = properties.history_len;
  this.showLegend = properties.show_legend;
  this.lineProperties = properties.line_properties;
  this.markerProperties = properties.marker_properties;

  // A list of plotted lines.
  this.paths = [];

  // A list of plotted markers.
  this.markers = [];

  // A map of data series, indexed by keys.
  this.histories = {};

  this.backgrounds = [];

  this.dynamicPolygons = {};
  BasePlot.call(this, properties);

  var obj = this;
  this.lineCallback = d3.svg.line()
      .x(function(d) { return obj.x(d.x); })
      .y(function(d) { return obj.y(d.y); });

  this.markerXCallback = function(d) { return obj.x(d.x); };
  this.markerYCallback = function(d) { return obj.y(d.y); };

  this.initBackground();

  this.simpleGraph.redraw()();
}


/** Inherit BasePlot. */
Sketch2DWindow.prototype = Object.create(BasePlot.prototype);


/** Set the constructor. */
Sketch2DWindow.prototype.constructor = Sketch2DWindow;


/**
 * Plot a polygon
 *
 * @param {list} xCoords A list of X coordinates of the polygon vertices.
 * @param {list} yCoords A list of Y coordinates of the polygon vertices.
 * @return {list} A list of vertices, each is a hashmap of x and y coordinates.
 */
Sketch2DWindow.prototype.plotPolygon = function(xCoords, yCoords) {
  var coords = [];

  if (xCoords.length != yCoords.length) {
    throw 'The sizes of X and Y arrays must be the same to define a polygon.';
  }

  for (var c = 0; c < xCoords.length; c++) {
    coords.push({'x': xCoords[c], 'y': yCoords[c]});
  }

  return coords;
};


/** Initialize the background. */
Sketch2DWindow.prototype.initBackground = function() {
  this.backgrounds = [];
  for (var i = 0; i < this.polygons.length; i++) {
    var xCoords = this.polygons[i]['x'];
    var yCoords = this.polygons[i]['y'];
    var color = this.polygons[i]['color'];
    var coords = this.plotPolygon(xCoords, yCoords);
    var path = this.plotLine(coords, color, 2, 'background', this.lineCallback);
    this.backgrounds.push(path);
    path.attr('d', this.lineCallback);
  }
};


/**
 * Update the plot when new data arrives from periodic AJAX polls.
 *
 * @param {hashmap} resp Response from the periodic data poll.
 * @param {bool} resetUponUpdate Reset the graph when new data is received.
 */
Sketch2DWindow.prototype.update = function(resp, resetUponUpdate) {
  var values = nestedIndex(resp, this.seriesSrcs);
  if (isBadType(values)) {
    return;
  }

  var keys = Object.keys(values);
  keys.sort();

  var respMarkers = nestedIndex(resp, this.markersSrcs);
  // Remove each marker from the graph.
  // TODO: rather than removing, just update the properties of the
  // existing markers, if they still exist.
  this.markers.forEach(function(marker) { marker[0][0].remove(); });
  // Empty the list.
  this.markers = []

  // Add new markers, if any.
  // TODO: If marker is outside of graph limits, then show an arrow
  // at the edge in the direction of the marker.
  if (this.markersSrcs) {
    var count = 0;
    Object.entries(respMarkers).forEach(([key, marker]) => {
      var property = this.markerProperties[key];
      var coords = [{'x': marker['x'], 'y': marker['y']}];
      this.markers.push(this.plotMarkers(
          coords, property['color'], this.markerXCallback, this.markerYCallback,
          property['size'], key, count++));
    });
  }

  for (i = 0; i < keys.length; i++) {
    var key = keys[i];
    if (!(key in this.histories)) {
      var property = this.lineProperties[key];
      var coords = [];
      this.histories[key] = coords;
      var path = this.plotLine(coords, property['color'], 4, 'series', null,
                               key, i);
      this.paths[key] = path;
    }
  }

  var respDynamicPolygons = nestedIndex(resp, this.dynamicsSrcs);
  if (this.dynamicPolygons) {
    // Remove dynamic polygons that are no longer being updated.
    for (var key in Object.keys(this.dynamicPolygons)) {
      if (this.dynamicPolygons.hasOwnProperty(key)) {
        if (isBadType(respDynamicPolygons) ||
            !respDynamicPolygons.hasOwnProperty(key)) {
          this.dynamicPolygons[key]['path'].remove();
          delete this.dynamicPolygons[key];
        }
      }
    }
  }
  if (!isBadType(respDynamicPolygons) && respDynamicPolygons) {
    for (var key in respDynamicPolygons) {
      if (respDynamicPolygons.hasOwnProperty(key)) {
        var xCoords = respDynamicPolygons[key]['x'];
        var yCoords = respDynamicPolygons[key]['y'];
        var color = respDynamicPolygons[key]['color'];
        var coords = this.plotPolygon(xCoords, yCoords);
        if (!this.dynamicPolygons.hasOwnProperty(key)) {
          this.dynamicPolygons[key] = {
              'path': this.plotLine(
                  coords, color, 2, 'background', this.lineCallback),
              'coords': coords
          };
        } else {
          var oldCoords = this.dynamicPolygons[key]['coords'];
          // Empty the old points and add the new points to the same array.
          oldCoords.splice(0, oldCoords.length);
          Array.prototype.push.apply(oldCoords, coords);
        }
        this.dynamicPolygons[key]['path'].attr('d', this.lineCallback);
      }
    }
  }

  if (resetUponUpdate) {
    this.simpleGraph.updateScale();
  }

  this.refresh(values);

  if (this.showLegend) {
    this.drawLegend(resetUponUpdate);
  }
};


/**
 * Update existing lines.
 *
 * @param {hashmap} values New points by series names.
 *     E.g. {'A': {'x': 0, y': 1.0}, 'B': {'x': 3.0, 'y': 2.0'}}
 */
Sketch2DWindow.prototype.refresh = function(values) {
  var keys = Object.keys(this.histories);

  for (var i = 0; i < keys.length; i++) {
    var key = keys[i];
    var path = this.paths[key];
    var coords = this.histories[key];
    if (values && key in values && !(isBadType(values[key]))) {
      var value = values[key];
      coords.push({'x': value['x'], 'y': value['y']});
    }
    while (coords.length > this.historyLen) {
      coords.shift();
    }

    path.attr('d', this.lineCallback);
  }

  for (var i = 0; i < this.backgrounds.length; i++) {
    this.backgrounds[i].attr('d', this.lineCallback);
  }
}
