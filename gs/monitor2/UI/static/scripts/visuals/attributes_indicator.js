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

/* Plot an indicator that can be expanded to show more details. */


/**
 * @constructor
 * @param {hashmap} properties JSON object that configures the indicator.
 */
function AttributesIndicator(properties) {
  this.name = properties.name;
  this.contentSrc = properties.src + '.content';
  this.indicatorSrc = properties.src + '.stoplight';
  if ('precision' in properties && properties.precision != null) {
    this.precision = properties.precision;
  } else {
    this.precision = 0;
  }
  var id = properties.id;
  this.valuesId = id + '_values';
  this.headerId = id + '_header';

  this.selector = '#' + id;
  this.valuesSelector = '#' + this.valuesId;
  this.headerSelector = '#' + this.headerId;

  this.render();

  var value_elem = $(this.valuesSelector);
  this.toggleDetails = function() {
    if (value_elem.is(':visible')) {
      value_elem.hide();
      this.renderText(this.valuesId, '');
    } else {
      value_elem.show();
    }
  };
  $(this.headerSelector).click(this.toggleDetails);
}


/** Inherit BaseIndicator. */
AttributesIndicator.prototype = Object.create(BaseIndicator.prototype);


/** Set the constructor. */
AttributesIndicator.prototype.constructor = AttributesIndicator;


/** Render the indicators with empty content. */
AttributesIndicator.prototype.render = function() {
  var html = '<div align="left">';
  html += '<span class="col span_' + gCanvasColumns + '_of_' + gCanvasColumns +
          '" style="border:0;margin:0;background-color:lightgray" ' +
          'id="' + this.headerId + '">';
  html += this.name;
  html += '</span>';
  html += '<span class="col span_12_of_12" style="font-size:8px;' +
          this.defaultStyle + '" id="' + this.valuesId + '">';
  html += '</span>';
  html += '</div>';
  $(this.selector).html(html);
  $(this.valuesSelector).hide();
};


/** Populate fields each with its own stoplight.
 *
 * @param {hashmap} fields A dictionary of fields, each has properties
 *     `value` and `stoplight`.
 *     Example: {'Voltage': {'stoplight': error, 'value': '12v'}}
 * @param {string} delimiter The HTML content between every two fields.
 * @return {string} The HTML content displaying the fields.
 */
AttributesIndicator.prototype.fieldsWithStoplight = function(fields,
                                                             delimiter) {
  var content = '';
  for (var key in fields) {
    if (fields.hasOwnProperty(key)) {
      var field = fields[key];
      if (field.hasOwnProperty('stoplight')) {
        content += '<span style="background-color:' +
                   getIndicatorColor(field.stoplight) + '">';
      }
      content += key;

      if (field.hasOwnProperty('value')) {
        content += ':&nbsp;';
        var value = field.value;
        if (isArray(value)) {
          for (var i = 0; i < value.length; i++) {
            content += valueToString(value[i], this.precision);
            if (i < value.length - 1) {
              content += ',&nbsp;';
            }
          }
        } else {
          content += valueToString(value, this.precision);
        }
      }

      if (field.hasOwnProperty('stoplight')) {
        content += '</span>';
      }
      content += delimiter;
    }
  }

  return content;
};


/**
 * Update indicator content to the following format:
 *     <indicator_name>
 *         <attribute0>: <field0>; <field1>; ...
 *         <attribute1>: ...
 *
 * @param {hashmap} resp Response from the periodic data poll, containing:
 *     'stoplight' {integer} The stoplight to use.
 *     'content' {array} A list of attributes, each is a hashmap containing:
 *         'name' {string} Optional name of the attribute to be displayed.
 *         'stoplight' {integer} The stoplight of the attribute.
 *         'fields' {hashmap} Optional fields of this attribute, keys are field
 *             names and values are hashmaps containing:
 *             'stoplight' {integer} The stoplight of the field.
 *             'value' A string, number, or array.
 *
 * Example: Given the following `resp` and an indicator name of 'MotorSbo',
 *     {
 *        'stoplight': ERROR,
 *        'content': [
 *            {
 *                'stoplight': ERROR,
 *                'name': 'Dynamics',
 *                'fields': {
 *                    'Voltages': {
 *                        'stoplight': ERROR,
 *                        'value': [10, 80, 300],
 *                    },
 *                    'Speed': {
 *                        'stoplight': GOOD,
 *                        'value': '150 rad/s',
 *                    }
 *                },
 *            },
 *        ]
 *     }
 * It produces the following once the user expands 'MotorSbo'.
 *    MotorSbo (RED)
 *        Dynamics: Voltages: 10, 80, 300; Speed: 150 rad/s;
 *       |  RED   |          RED         |     GREEN       |
 */
AttributesIndicator.prototype.update = function(resp) {
  this.refreshColor(resp);

  if (!$(this.valuesSelector).is(':visible')) {
    return;
  }

  var data = nestedIndex(resp, this.contentSrc);
  if (data == null) {
    if (!$(this.valuesSelector).is(':empty')) {
      this.renderText(this.valuesId, '');
    }
    return;
  }

  var content = '';
  // Iterate through all attributes.
  for (var i = 0; i < data.length; i++) {
    var attributeName = data[i].name;
    if (i != 0) {
      content += '<br/>';
    }

    if (attributeName) {
      if (isNumber(data[i].stoplight)) {
        content += '<span style="background-color:' +
                   getIndicatorColor(data[i].stoplight) + '">';
      }
      content += attributeName;
      if (isNumber(data[i].stoplight)) {
        content += '</span>';
      }
    }
    if (data[i].fields) {
      // If attribute name exists, render it as
      // "<attribute_name>: <field0>; <field1>; ...".
      // Otherwise, render it as "<field0>; <field1>; ...".
      if (attributeName) {
        content += ':&nbsp;';
      }
      var delimiter = ';&nbsp;';
      content += this.fieldsWithStoplight(data[i].fields, delimiter);
    }
  }
  $(this.valuesSelector).html(content);
};
