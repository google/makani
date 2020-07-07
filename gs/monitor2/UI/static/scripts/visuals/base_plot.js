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
 * The default list of colors.
 * @const
 */
var DEFAULT_COLORS = [
  '#2A17B1', /* blue */
  '#41DB00', /* green */
  '#EF002A', /* red */
  '#FFD500', /* brownyellow */
  'magenta',
  'purple',
  'orange',
  'steelblue',
  'lawngreen',
  'salmon',
  'brown',
  'yellow'
];


/**
 * @constructor Base plotting class.
 *
 * @param {hashmap} properties An object with following attributes:
 *     x_attrs and y_attrs are parameters defining the X and Y axes properties,
 *         respectively. Each of them contains:
 *         domain: Optional axis domain. [lowerBound, upperBound]
 *         label: Label of the axis.
 *         src: Index to the field that contains the coordinate of the new
 *             data point.
 *     selector is the ID of the DOM element containing the plot area.
 */
function BasePlot(properties) {
  this.autoYDomain = properties.y_attrs.domain ? false : true;
  this.xSrc = properties.x_attrs.src;
  this.ySrc = properties.y_attrs.src;

  // SimpleGraph is a third-party library to draw 2D lines with a canvas
  // that is zoomable and pannable.
  // Reference: http://bl.ocks.org/stepheneb/1182434
  this.simpleGraph = new SimpleGraph(properties, this);
  this.svg = this.simpleGraph.svg;
  this.x = this.simpleGraph.x;
  this.y = this.simpleGraph.y;
}


/**
 * Automatically update domain of the Y axis.
 *
 * @param {list} yCoords A list of values to span across the domain.
 * @param {number} pastYMin The minimum Y coordinate from the past.
 * @param {number} pastYMax The maximum Y coordinate from the past.
 */
BasePlot.prototype.autoUpdateYDomain = function(yCoords, pastYMin, pastYMax) {
  if (!this.autoYDomain) {
    return;
  }

  if (!this.autoUpdatedYDomain) {
    this.autoUpdatedYDomain = true;
  }

  var bound = d3.extent(yCoords);
  if (isBadNumber(bound[0]) || isBadNumber(bound[1])) {
    return;
  }

  var lowerBound, upperBound;
  if (isBadNumber(pastYMin)) {
    lowerBound = bound[0];
  } else {
    lowerBound = Math.min(bound[0], pastYMin);
  }

  if (isBadNumber(pastYMax)) {
    upperBound = bound[1];
  } else {
    upperBound = Math.max(bound[1], pastYMax);
  }

  var currentLimits = this.simpleGraph.getYLimits();

  // Leave some cushion space for better visualization.
  var margin = 0.01;
  lowerBound -= Math.abs(lowerBound * 0.25) + margin;
  upperBound += Math.abs(upperBound * 0.2) + margin;

  if (currentLimits[0] != lowerBound || currentLimits[1] != upperBound) {
    this.simpleGraph.setYDomain([lowerBound, upperBound]);
    this.simpleGraph.redraw()();
  }
};


/** Draw legends.
 *
 * @param {bool} refresh True if legend should be redrawn even if one exists.
 */
BasePlot.prototype.drawLegend = function(refresh) {
  if (!refresh && this.legend) {
    // Legend already exists and need not update.
    return;
  }

  // Remove old legend if one exists.
  if (this.legend) {
    this.legend.remove();
  }

  // Draw the legend.
  this.legend = this.svg.append('g')
    .attr('class', 'legend')
    .attr('transform', 'translate(10, 10)')
    .style('font-size', '12px')
    .call(d3.legend);
};


/**
 * Plot a line with D3.
 *
 * @param {list} coordinates A list of coordinates.
 * @param {string} strokeColor Color of the line.
 * @param {number} strokeWidth Width of the line.
 * @param {string} lineClass CSS class of the line.
 * @param {function} lineCallback The call back function to return x and y
 *     coordinates given an object in `coordinates`. An example would be:
 *     lineCallback = d3.svg.line()
 *         .x(function(d) { return d.x; } )
 *         .y(function(d) { return d.y; } );
 * @param {string} name Legend of the line.
 * @param {number} legendOrder ID of the line used to sort legends.
 * @return {svg element} line A line object.
 */
BasePlot.prototype.plotLine = function(
    coordinates, strokeColor, strokeWidth, lineClass, lineCallback, name,
    legendOrder) {

  var line = this.svg.append('path')
        .datum(coordinates)
        .attr('class', lineClass)
        .attr('stroke', strokeColor)
        .attr('stroke-width', strokeWidth)
        .attr('fill', 'none')
        .attr('d', lineCallback);
  if (name != null) {
    line.attr('data-legend', name);
  }
  if (legendOrder != null) {
    line.attr('data-legend-pos', legendOrder);
  }
  return line;
};


/**
 * Plot point markers with D3.
 *
 * @param {list} coordinates A list of coordinates.
 * @param {string} strokeColor Color of the marker.
 * @param {function} markerXCallback The call back function to return x
 *     coordinates given an object in `coordinates`. An example would be:
 *     callback = function(d) {return x_scale(d.x);};
 * @param {function} markerYCallback Similar to markerXCallback but for Y.
 * @param {number} markerRadius Radius of the marker.
 * @param {string} name Legend of the marker.
 * @param {number} legendOrder ID of the marker used to sort legends.
 * @return {svg element} marker A marker object.
 */
BasePlot.prototype.plotMarkers = function(
    coordinates, strokeColor, markerXCallback, markerYCallback, markerRadius,
    name, legendOrder) {
  var markers = this.svg.selectAll('dot')
    .data(coordinates)
    .enter().append('circle')
        .attr('r', markerRadius)
        .attr('stroke', strokeColor)
        .attr('cx', markerXCallback)
        .attr('cy', markerYCallback);

  if (name != null) {
    markers.attr('data-legend', name);
  }
  if (legendOrder != null) {
    markers.attr('data-legend-pos', legendOrder);
  }
  markers.attr('data-legend-color', strokeColor);

  return markers;
};


/**
 * Plot a circle with D3.
 *
 * @param {number} xCoord The x coordinate of the circle.
 * @param {number} yCoord The y coordinate of the circle.
 * @param {number} radius The radius of the circle.
 * @param {string} color The color of the circle.
 * @return {svg element} circle A circle object.
 */
BasePlot.prototype.plotCircle = function(xCoord, yCoord, radius, color) {
  var circle = this.svg.append('circle')
       .attr('cx', this.x(xCoord))
       .attr('cy', this.y(yCoord))
       .attr('stroke', color)
       .attr('fill', 'white')
       .attr('r', radius);
  return circle;
};


/**
 * Update the plot when new data arrives from periodic AJAX polls.
 *
 * @param {hashmap} resp Response from the periodic data poll.
 * @param {bool} resetUponUpdate Reset the graph when new data is received.
 */
BasePlot.prototype.update = function(resp, resetUponUpdate) {
  throw 'BasePlot::update is not implemented.';
};


/**
 * Update existing lines.
 *
 * The function updates all plots (i.e., paths) by calling
 * path.attr('d', lineCallback). The function is called in two cases:
 * 1. When the graph is scaled/panned, the function is called with no arguments.
 * 2. When the data is updated, the function is called with arguments about
 *     new data. The list of arguments differ across derived plots.
 */
BasePlot.prototype.refresh = function() {
  throw 'BasePlot::refresh is not implemented.';
};
