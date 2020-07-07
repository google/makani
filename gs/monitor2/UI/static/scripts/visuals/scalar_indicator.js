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

/* Plot a scalar indicator. */


/**
 * @constructor
 * @param {hashmap} properties JSON object that configures the indicator.
 */
function ScalarIndicator(properties) {
  BaseIndicator.call(this, properties);

  this.renderIndicator('');
  // Cached value.
  this.value = null;
  // Cached text.
  this.text = null;
}


/* Inherit BaseIndicator. */
ScalarIndicator.prototype = Object.create(BaseIndicator.prototype);


/** Set the constructor. */
ScalarIndicator.prototype.constructor = ScalarIndicator;


/**
 * Update indicator values.
 *
 * @param {hashmap} resp Response from the periodic data poll.
 */
ScalarIndicator.prototype.update = function(resp) {
  this.refreshColor(resp);

  var value = nestedIndex(resp, this.src);
  // Skip if the raw values remains the same.
  if (value != this.value) {
    this.value = value;
    var text = valueToString(value, this.precision);
    // Skip if the text output remains the same.
    if (text != this.text) {
      this.text = text;
      this.renderText(this.valuesId, text);
    }
  }
}
