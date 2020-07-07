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

/* Plot a dictionary of values in an indicator. */


/**
 * @constructor
 * @param {hashmap} properties JSON object that configures the indicator.
 */
function DictIndicator(properties) {
  BaseIndicator.call(this, properties);

  // Initialize the content.
  this.renderIndicator('');

  if ('secondary_keys' in properties && properties.secondary_keys) {
    this.secondaryKeys = properties.secondary_keys;
  } else {
    this.secondaryKeys = null;
  }

  if ('heading' in properties && properties.heading) {
    this.heading = properties.heading;
  } else {
    this.heading = null;
  }

  if (this.layout == 'table' && this.keys) {
    this.renderTableFrame(this.keys);
  }

  // Cached data values.
  this.rawDataStr = {};
}


/** Inherit BaseIndicator. */
DictIndicator.prototype = Object.create(BaseIndicator.prototype);


/** Set the constructor. */
DictIndicator.prototype.constructor = DictIndicator;


/**
 * Update indicator values.
 *
 * @param {hashmap} resp Response from the periodic data poll.
 */
DictIndicator.prototype.update = function(resp) {
  this.refreshColor(resp);
  this.refreshMessage(resp);

  var data = nestedIndex(resp, this.src);
  if (data == null) {
    if (!$(this.valuesSelector).is(':empty')) {
      if (this.layout == 'table' && this.keys) {
        this.renderTableText(this.keys);
      } else {
        this.renderText(this.valuesId, '');
      }
    }
    return;
  }

  if (this.keys == null) {
    this.keys = Object.keys(data);
    this.keys.sort();
    this.max_key_len = this.keys.reduce(
        function(a, b) {return a.length > b.length ? a : b;}).length;
    if (this.layout == 'table') {
      this.renderTableFrame(this.keys);
    }
  }

  if (this.secondaryKeys) {
    var rawKeys = [];
    for (var i = 0; i < this.keys.length; i++) {
      for (var j = 0; j < this.secondaryKeys.length; j++) {
        rawKeys.push(this.keys[i] + '.' + this.secondaryKeys[j]);
      }
    }
  } else {
    var rawKeys = this.keys;
  }
  var rawDataStr = hashOfStringsFromDict(rawKeys, data, this.precision);

  if (!equalHashmaps(rawDataStr, this.rawDataStr, rawKeys)) {
    // Refresh HTML only if data is updated.
    this.rawDataStr = rawDataStr;

    if (this.secondaryKeys) {
      var values = {};
      for (var i = 0; i < this.keys.length; i++) {
        var items = [];
        for (var j = 0; j < this.secondaryKeys.length; j++) {
          items.push(rawDataStr[this.keys[i] + '.' + this.secondaryKeys[j]]);
        }
        var content = items[0];
        for (var j = 1; j < this.secondaryKeys.length; j++) {
          content += ' ' + items[j];
        }
        values[this.keys[i]] = content;
      }
    } else {
      values = rawDataStr;
    }

    if (this.layout == 'table') {
      this.renderTableText(this.keys, values);
    } else {
      var content = this.renderPlainText(this.mode, this.keys, values,
                                     this.max_key_len);
      this.renderText(this.valuesId, content);
    }
  }
}
