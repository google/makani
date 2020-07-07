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

/* Utilities for numerical operations and struct indexing. */


/* Test if the variable is a string. */
function isString(value) {
  return typeof value == 'string' || value instanceof String;
}


/* Test if the variable is a string. */
function isNumber(value) {
  return typeof value == 'number';
}


/* Test if the variable is an array. */
function isArray(value) {
  return value instanceof Array;
}


function valueToString(value, precision) {
  var default_len = 6;
  if (isNumber(value)) {
    return toFixed(value, precision).padLeft(default_len);
  } else if (isArray(value)) {
    var new_array = [];
    for (i = 0; i < value.length; i++) {
      var elem = value[i];
      if (isNumber(elem)) {
        elem = toFixed(elem, precision).padLeft(default_len);
      }
      new_array.push(elem);
    }
    return new_array.join();
  } else if (isString(value)) {
    return value.padLeft(default_len);
  } else {
    return value;
  }
}


/* Round a number to a given precision. */
function toFixed(value, precision) {
  var power = Math.pow(10, precision || 0);
  return (Math.round(value * power) / power).toFixed(precision);
}


/* Check whether value is invalid. */
function isBadType(value) {
  return typeof(value) == 'undefined' || value == null;
}


/* Check whether value is invalid. */
function isBadNumber(value) {
  return typeof(value) == 'undefined' || value == null || isNaN(value);
}


/**
 * Define a mod function that always returns positive values.
 *
 * @param {float} a The dividend.
 * @param {float} b The divisor.
 * @return {float} The residue.
 */
Math.fmod = function(a, b) {
  return Number((a - (Math.floor(a / b) * b)).toPrecision(8));
};


/* Wrap around a number within bounds. */
function wrap(value, bounds) {
  var left = bounds[0];
  var right = bounds[1];
  var wrap0 = Math.fmod(value - left, right - left);
  if (wrap0 >= 0.) {
    return wrap0 + left;
  } else {
    return wrap0 + right;
  }
}


/* Index a nested struct with a string of indices concatenated by '.'. */
function nestedIndex(structObj, index) {
  if (isBadType(index) || isBadType(structObj)) {
    return null;
  }
  var delimiter = '.';
  var indices = index.split(delimiter);
  var value = structObj;
  for (var i = 0; i < indices.length; i++) {
    var idx = indices[i];
    if (!isBadType(value) && idx in value) {
      value = value[idx];
    } else {
      return null;
    }
  }
  return value;
}


/** Construct a hashmap of stringified values from a dict of values and keys.
 *
 * @param {array} keys A list of keys.
 * @param {hashmap} data A dict of values.
 * @param {integer} precision An integer specifying the number of digits after
 *     the decimal.
 * @return {hashmap} The hashmap of key/value pairs.
 */
function hashOfStringsFromDict(keys, data, precision) {
  var new_data = {};
  for (var i = 0; i < keys.length; i++) {
    var key = keys[i];
    var value;
    if (key in data && data[key] != null) {
      new_data[key] = valueToString(data[key], precision);
    } else {
      new_data[key] = valueToString('--');
    }
  }
  return new_data;
}


/** Construct a hashmap of stringified values from a list of values and keys.
 *
 * @param {array} keys A list of keys.
 * @param {array} values A list of values.
 * @param {integer} precision An integer specifying the number of digits after
 *     the decimal.
 * @return {hashmap} The hashmap of key/value pairs.
 */
function hashOfStringsFromList(keys, values, precision) {
  var isCorrupted = (keys.length != values.length);

  var data = {};
  for (var i = 0; i < keys.length; i++) {
    var key = keys[i];
    if (isCorrupted) {
      data[key] = valueToString('--');
    } else {
      data[key] = valueToString(values[i], precision);
    }
  }
  return data;
}


/** Check whether two hashmaps are equal with respect to the given set of keys.
 *
 * @param {hashmap} map_a The first hashmap.
 * @param {hashmap} map_b The second hashmap.
 * @param {array} keys The keys for which the two hashmaps must agree.
 * @return {bool} True if the hashmaps are equal.
 */
function equalHashmaps(map_a, map_b, keys) {
  var isChanged = false;
  for (var i = 0; i < keys.length; i++) {
    var key = keys[i];
    var key_in_map_a = (typeof map_a[key] != 'undefined');
    var key_in_map_b = (typeof map_b[key] != 'undefined');
    if (!key_in_map_a && !key_in_map_b) {
      continue;
    } else if (!(key_in_map_a && key_in_map_b && map_a[key] == map_b[key])) {
      isChanged = true;
      break;
    }
  }
  return !isChanged;
}


/**
 * Update the min and max according to coordinate values along an axis.
 *
 * @param {array} coords An array of coordinates, each has one or more axes.
 * @param {number} min The current minimum value.
 * @param {number} max The current maximum value.
 * @param {string} axis The name of the axis.
 *
 * @return {hashmap} The updated min and max values.
 */
function updateBound(coords, min, max, axis) {
  var values = coords.map(function(point) {return point[axis];});
  if (values) {
    var bound = d3.extent(values);
    if (!isBadNumber(bound[0])) {
      min = Math.min(min, bound[0]);
    }
    if (!isBadNumber(bound[1])) {
      max = Math.max(max, bound[1]);
    }
  }
  return {
      'min': min,
      'max': max,
  };
}
