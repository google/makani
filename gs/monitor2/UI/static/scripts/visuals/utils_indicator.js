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

/* Utilities to plot indicators.
 *
 * An indicator is a dictionary or list of itemized values.
 * Each indicator has a stoplight that can be green, yellow, or red.
 */


var gCanvasColumns = 12;


/**
 * Get indicator's color by value.
 *
 * @param {int} indicatorValue Value of the indicator.
 * @return {string} Color of the indicator.
 */
function getIndicatorColor(indicatorValue) {
  if (indicatorValue == gStoplightValues.error) {
    return 'salmon';
  } else if (indicatorValue == gStoplightValues.normal) {
    return 'lightgreen';
  } else if (indicatorValue == gStoplightValues.warning) {
    return 'yellow';
  } else if (indicatorValue == gStoplightValues.unavailable) {
    return 'lightgray';
  } else {
    return '#eee';
  }
}


/**
 * @constructor Base indicator class.
 *
 * @param {object} properties Options for plotting, which should contain:
 *     'precision: The precision of the values to show.
 *     'keys': The name of the fields.
 *     'mode': It can be either 'horizontal' or 'vertical', which defines the
 *         orientation in which values are shown.
 *     'layout': It can be either 'table' or 'plain', which specifies how
 *         values are organized.
 */
function BaseIndicator(properties) {
  this.name = properties.name;
  this.src = properties.src;
  this.indicatorSrc = properties.indicator_src;
  this.message = '';
  this.messageSrc = properties.message_src;
  if ('precision' in properties && properties.precision != null) {
    this.precision = properties.precision;
  } else {
    this.precision = 2;
  }
  if ('keys' in properties && properties.keys) {
    this.keys = properties.keys;
    this.max_key_len = this.keys.reduce(
        function(a, b) {return a.length > b.length ? a : b;}).length;
  } else {
    this.keys = null;
  }
  if ('mode' in properties) {
    this.mode = properties.mode;
  } else {
    this.mode == 'horizontal';
  }
  if ('layout' in properties && properties.layout != null) {
    this.layout = properties.layout;
  } else {
    this.layout = 'table';
  }

  var id = properties.id;

  this.valuesId = id + '_values';
  this.headerId = id + '_header';
  this.messageId = id + '_msg';
  this.cellId = id + '_cell';
  this.selector = '#' + id;
  this.valuesSelector = '#' + this.valuesId;
  this.headerSelector = '#' + this.headerId;
  this.defaultStyle = 'padding:0;border:0;margin:0;word-wrap:break-word;' +
      'white-space:pre-wrap';
  this.fontSize = properties.font_size;

  // Cached stoplight color.
  this.color = null;
}


/** Render an empty HTML table where content will be added later.
 * @param {array} keys A list of keys.
 */
BaseIndicator.prototype.renderTableFrame = function(keys) {
  var innerValuesId = this.valuesId + '_inner';
  var content = '';

  var body = '';
  var contentStyle = this.defaultStyle;
  if (!isBadNumber(this.fontSize)) {
    contentStyle += ';font-size:' + this.fontSize + 'px';
  }
  if (keys) {
    // Evenly assign grid width (gCanvasColumns) to each column.
    var col_width = gCanvasColumns / keys.length;
    for (i = 0; i < keys.length; i++) {
      var value = valueToString('--');
      var payload = keys[i].padLeft(this.max_key_len) +
          ':<span id="' + this.cellId + i + '">' + value;
      if (this.mode == 'horizontal') {
        body += '<td style="' + contentStyle + '" ' +
            'class="col span_' + col_width + '_of_' + gCanvasColumns + '">' +
            payload + '</td>';
      } else {
        body += '<tr><td style="' + contentStyle + '">' +
            payload + '</td></tr>';
      }
    }
  }

  // Add the message at the end so the above numbers do not flicker.
  if (this.heading) {
    content += this.heading + '<br/>';
  }

  if (this.mode == 'horizontal') {
    content += '<table style=' +
        '"display:inline-block;' + contentStyle + '">' +
        '<tbody><tr id="' + innerValuesId + '">';
    content += body + '</tr></tbody></table>';
  } else {
    content += '<table style="' + contentStyle + '">' +
        '<tbody id="' + innerValuesId + '">';
    content += body + '</tbody></table>';
  }

  // Add the message at the end so the above numbers do not flicker.
  if (this.messageSrc) {
    content += '<div id="' + this.messageId + '"></div>';
  }

  $(this.valuesSelector).html(content);

  this.valuesSelector = '#' + innerValuesId;
};


/**
 * Render the frame of the indicator.
 *
 * @param {string} content The content text of the indicator.
 */
BaseIndicator.prototype.renderIndicator = function(content) {
  var headerCols;
  var contentCols;
  var paddingLeft;
  if (this.mode == 'horizontal') {
    // Split the gCanvasColumns columns amongst header and content.
    headerCols = gCanvasColumns / 3;
    contentCols = gCanvasColumns - headerCols;
    paddingLeft = 10;
  } else {
    // Both header and content occupy the full width.
    headerCols = gCanvasColumns;
    contentCols = gCanvasColumns;
    paddingLeft = 0;
  }
  var html = '<div align="left"><span class="col span_' +
      headerCols + '_of_' + gCanvasColumns + '" style=' +
      '"background-color:lightgray;' + this.defaultStyle + '"' +
      ' id="' + this.headerId + '">' + this.name + '</span>';
  html += '<span class="col span_' + contentCols + '_of_' + gCanvasColumns +
      '" ' + 'style="text-align:left;' + this.defaultStyle + '">' +
      '<div style="padding-left:' + paddingLeft;

  if (!isBadNumber(this.fontSize)) {
    html += ';font-size:' + this.fontSize + 'px';
  }

  html += '" id="' + this.valuesId + '">' + content + '</div></span>';
  html += '</div>';
  $(this.selector).html(html);
};


/**
 * Refresh the color of the indicator.
 *
 * @param {object} resp Data response from the server.
 */
BaseIndicator.prototype.refreshColor = function(resp) {
  var indicatorValue = nestedIndex(resp, this.indicatorSrc);
  var color = getIndicatorColor(indicatorValue);
  if (color != this.color) {
    this.color = color;
    $(this.headerSelector).attr(
        'style',
        this.defaultStyle + ';background-color:' + color);
  }
};


/**
 * Refresh the message for the indicator.
 *
 * @param {object} resp Data response from the server.
 */
BaseIndicator.prototype.refreshMessage = function(resp) {
  if (this.messageSrc) {
    var message = nestedIndex(resp, this.messageSrc);
    if (message != this.message) {
      this.renderText(this.messageId, message);
      this.message = message;
    }
  }
};


/**
 * Render values in HTML table.
 *
 * @param {array} keys A list of keys.
 * @param {object} data A dict of values for each key.
 */
BaseIndicator.prototype.renderTableText = function(keys, data) {
  var num_keys = keys.length;
  var value;
  for (var i = 0; i < num_keys; i++) {
    if (data && keys[i] in data) {
      value = data[keys[i]];
    } else {
      value = '';
    }
    this.renderText(this.cellId + i, value);
  }
};


/**
 * Render values in HTML table.
 *
 * @param {string} id Id of the DOM element.
 * @param {string} text String to display.
 */
BaseIndicator.prototype.renderText = function(id, text) {
  var elem = document.getElementById(id);
  if (elem) {
    elem.innerText = text;
  }
};


/**
 * Render values in plain text.
 *
 * @param {string} mode 'horizontal' or 'vertical' layout.
 * @param {array} keys A list of keys.
 * @param {object} data A dict of values for each key.
 * @param {integer} max_key_len Length of the longest key.
 *
 * @return {string} HTML for the displayed table.
 */
BaseIndicator.prototype.renderPlainText = function(mode, keys, data,
                                                   max_key_len) {
  var content = '';
  for (i = 0; i < keys.length; i++) {
    var key = keys[i];
    var value = data[key];
    var payload = keys[i].padLeft(max_key_len) + ':' + value;
    var tab_length = 8;
    if (mode == 'horizontal') {
      content += payload;
      var remainder = (payload.length) % tab_length;
      if (remainder == 0)
        remainder = tab_length;
      for (j = 0; j < remainder; j++) {
        content += ' ';
      }
    } else {
      content += payload + '\n';
    }
  }
  return content;
};
