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

/* Utilities to plot a tape indicator.
 *
 * An indicator is a dictionary or list of itemized values.
 * Each indicator has a stoplight that can be green, yellow, or red.
 */


/**
 * @constructor Base indicator class.
 *
 * @param {object} properties Options for plotting, which should contain:
 *     'srcPosition': Position of the bar.
 *     'srcAnnotation': Text annotating the bar value.
 *     'aspectRatio': The width:height aspect ratio of the indicator.
 *     'id': Id of the containing DOM element.
 *     Other fields defining the appearance, see createTapeIndicator.
 */
function TapeIndicator(properties) {
  this.srcPosition = properties.srcPosition;
  this.srcAnnotation = properties.srcAnnotation;

  this.chart = document.getElementById(properties.id);
  properties.width = this.chart.clientWidth;
  properties.height = properties.aspectRatio ?
    this.chart.clientWidth / properties.aspectRatio :
    this.chart.clientHeight;

  // Three elements are needed to produce the left bar.
  // Id of the color-filled bar body.
  this.barBodyId = properties.id + '_bar_body';
  // The bar-top.
  this.barTopId = properties.id + '_bar_top';
  // The text above the bar.
  this.barTextId = properties.id + '_bar_text';
  this.barTopHeight = 5;

  this.createTapeIndicator(properties.id, properties);
}


/**
 * Update indicator values.
 *
 * @param {hashmap} resp Response from the periodic data poll.
 */
TapeIndicator.prototype.update = function(resp) {
  var position = nestedIndex(resp, this.srcPosition);
  var annotation = nestedIndex(resp, this.srcAnnotation);

  // Update the bar.
  d3.select('#' + this.barBodyId).style('top', this.barBodyPosition(position));
  d3.select('#' + this.barTopId).style('top', this.barTopPosition(position));
  d3.select('#' + this.barTextId)
      .style('bottom', this.barTextPosition(position))
      .text(annotation);
};


/**
 * Compute the top position of the bar's body.
 *
 * @param {float} position The value that the bar represents in the axis scale.
 # @return {string} Position in the pixel coordinate.
 */
TapeIndicator.prototype.barBodyPosition = function(position) {
  return this.yScale(position) + 'px';
};


/**
 * Compute the base position of the bar top.
 *
 * @param {float} position The value that the bar represents in the axis scale.
 # @return {string} Position in the pixel coordinate.
 */
TapeIndicator.prototype.barTopPosition = function(position) {
  return (this.yScale(position) - this.barTopHeight / 2) + 'px';
};


/**
 * Compute the base position for the bar's annotation text.
 *
 * @param {float} position The value that the bar represents in the axis scale.
 # @return {string} Position in the pixel coordinate.
 */
TapeIndicator.prototype.barTextPosition = function(position) {
  return (this.yScale(this.yLimits[0]) - this.yScale(position)) + 'px';
};


/**
 * This function creates tape indicator.
 *
 * @param {string} containerId The element that contains the indicator.
 * @param {hashmap} config Parameters. Example:
 *     {
 *         title: 'Sample 1!',
 *         aspectRatio: 1.8,
 *         leftBar: {
 *             color: '#EDB818',
 *             position: 0.89,
 *             title: '0.89'
 *         },
 *         rightBars: [
 *             {
 *                 position: 0.08,
 *                 textBelow: 'STOP'
 *             },
 *             {
 *                 position: 0.4,
 *                 textAbove: 'Low',
 *                 textBelow: 'Very Low'
 *             },
 *         ],
 *         yLimits: [0.0, 1.0],
 *         yTicks: [
 *             {
 *                 position: 0.08,
 *                 label: 'E-Stop)'
 *             },
 *             {
 *                 position: 0.2,
 *                 label: '0.2'
 *             }
 *         ]
 *     }
 */
TapeIndicator.prototype.createTapeIndicator = function(containerId, config) {
  var containerSelection = d3.select(
      document.querySelector('#' + containerId));
  containerSelection.append('div').text(config.title);
  var chartSelection = containerSelection.append('div')
          .style('position', 'relative')
          .style('width', config.width + 'px')
          .style('height', config.height + 'px')
          .style('border', '1px solid #000000');
  //Init scale to calculate position
  var yScale = d3.scale.linear()
          .range([config.height, 0])
          .domain(config.yLimits);
  this.yScale = yScale;
  this.yLimits = config.yLimits;

  //Right section drawing
  var rightBarWrapper = chartSelection.append('div')
          .classed('right-bar', true)
          .style('position', 'absolute')
          .style('border', '2px solid #000000')
          .style('left', '30%')
          .style('right', '20%')
          .style('top', 0)
          .style('background-color', '#eeeeee')
          .style('bottom', 0);

  var barEnter = rightBarWrapper.selectAll('.bar')
          .data(config.rightBars)
          .enter()
          .append('div')
          .classed('bar', true);
  barEnter.append('div')
          .classed('text-above', true);
  barEnter.append('div')
          .classed('text-below', true);
  rightBarWrapper.selectAll('.bar')
          .style('position', 'absolute')
          .style('left', 0)
          .style('right', 0)
          .style('top', function(item) {
              return yScale(item.position) + 'px';
          })
          .style('border-top', '2px solid #000000');
  rightBarWrapper.selectAll('.bar').select('.text-above')
          .style('position', 'absolute')
          .style('left', 0)
          .style('right', 0)
          .style('transform', 'translateY(-100%)')
          .style('-webkit-transform', 'translateY(-100%)')
          .style('-moz-transform', 'translateY(-100%)')
          .style('-ms-transform', 'translateY(-100%)')
          .style('-o-transform', 'translateY(-100%)')
          .style('text-align', 'center')
          .text(function(item) {
              return item.textAbove;
          });
  rightBarWrapper.selectAll('.bar').select('.text-below')
          .style('position', 'absolute')
          .style('left', 0)
          .style('right', 0)
          .style('top', '0')
          .style('text-align', 'center')
          .text(function(item) {
              return item.textBelow;
          });

  //Axis drawing
  var axisWrapper = chartSelection.append('div')
          .classed('axis', true);
  axisWrapper.selectAll('div')
          .data(config.yTicks)
          .enter()
          .append('div');
  axisWrapper.selectAll('div')
          .style('position', 'absolute')
          .style('left', '85%')
          .style('white-space', 'nowrap')
          .style('top', function(item) {
              return (yScale(item.position) - 5) + 'px';
          })
          .text(function(item) {
              return item.label;
          });

  //Left bar drawing
  var leftBarWrapper = chartSelection.append('div')
          .classed('left-bar', true)
          .style('position', 'absolute')
          .style('border', '2px solid #000000')
          .style('left', 0)
          .style('width', '30%')
          .style('top', 0)
          .style('bottom', 0)
          .style('border-right', '1px solid #000000');
  leftBarWrapper.append('div')
          .attr('id', this.barBodyId)
          .classed('bar', true)
          .style('position', 'absolute')
          .style('left', 0)
          .style('right', 0)
          .style('top', this.barBodyPosition(config.leftBar.position))
          .style('bottom', 0)
          .style('background-color', config.leftBar.color);
  leftBarWrapper.append('div')
          .attr('id', this.barTopId)
          .classed('level', true)
          .style('position', 'absolute')
          .style('left', '-5px')
          .style('right', '-5px')
          .style('top', this.barTopPosition(config.leftBar.position))
          .style('height', this.barTopHeight + 'px')
          .style('background-color', '#000000');
  leftBarWrapper.append('div')
          .attr('id', this.barTextId)
          .classed('title', true)
          .style('position', 'absolute')
          .style('left', 0)
          .style('right', 0)
          .style('text-align', 'center')
          .style('bottom', this.barTextPosition(config.leftBar.position))
          .text(config.leftBar.title);

};
