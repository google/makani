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

/* Plot a list of values in an indicator. */


/**
 * @constructor
 * @param {hashmap} properties JSON object that configures the indicator.
 */
function ListIndicator(properties) {
  BaseIndicator.call(this, properties);

  // Render the indicator with the initial content.
  this.renderIndicator('');

  if (this.layout == 'table') {
    this.renderTableFrame(this.keys);
  }

  // Cached data values.
  this.data = {};
}


/** Inherit BaseIndicator. */
ListIndicator.prototype = Object.create(BaseIndicator.prototype);


/** Set the constructor. */
ListIndicator.prototype.constructor = ListIndicator;


/**
 * Update indicator values.
 *
 * @param {hashmap} resp Response from the periodic data poll.
 */
ListIndicator.prototype.update = function(resp) {
  this.refreshColor(resp);
  this.refreshMessage(resp);

  var values = nestedIndex(resp, this.src);
  if (values == null) {
    if (!$(this.valuesSelector).is(':empty')) {
      if (this.layout == 'table') {
        this.renderTableText(this.keys);
      } else {
        this.renderText(this.valuesId, '');
      }
    }
    return;
  }

  var data = hashOfStringsFromList(this.keys, values, this.precision);
  if (!equalHashmaps(data, this.data, this.keys)) {
    // Refresh HTML only if data is updated.
    this.data = data;
    var content;
    if (this.layout == 'table') {
      this.renderTableText(this.keys, data);
    } else {
      content = this.renderPlainText(this.mode, this.keys, data,
                                     this.max_key_len);
      this.renderText(this.valuesId, content);
    }
  }
}
