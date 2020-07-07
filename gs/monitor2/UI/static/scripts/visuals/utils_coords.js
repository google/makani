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

/* Utilities to obtain X and Y coordinates. */


/**
 * Obtain Y coordinates that correspond to an X coordinate.
 *
 * @param {hashmap} resp The response object.
 * @param {string} xSrc Index to the X coordinate in the response.
 * @param {string} ySrc Index to Y coordinates in the response.
 * @param {list} xWrap A list of two values [lower_bound, upper_bound) to
 *     wrap-around X coordinates.
 * @param {list} yWrap A list of two values [lower_bound, upper_bound) to
 *     wrap-around Y coordinates.
 *
 * @return {hashmap} A hashmap with the following fields:
 *     originalX: A list of original X coordinate values.
 *     x: A list of processed X coordinate values.
 *     y: A list of processed Y coordinate values.
 */
function getYCoordListWithOneXCoord(resp, xSrc, ySrc, xWrap, yWrap) {
  var xCoord = nestedIndex(resp, xSrc);
  var yCoords = nestedIndex(resp, ySrc);
  if (xCoord == null || yCoords == null) {
    return null;
  }
  if (!Array.isArray(yCoords)) {
    // Convert to array if it is a scalar.
    yCoords = [yCoords];
  }
  var originalXCoord = xCoord;
  if (xWrap) {
    xCoord = wrap(xCoord, xWrap);
  }
  if (yWrap) {
    for (i = 0; i < yCoords.length; i++) {
      yCoords[i] = wrap(yCoords[i], yWrap);
    }
  }
  return {
    'originalX': originalXCoord,
    'x': xCoord,
    'y': yCoords
  };
}


/**
 * Obtain Y coordinates and X coordinates from a response object.
 *
 * @param {hashmap} resp The response object.
 * @param {string} xSrc Index to X coordinates in the response.
 * @param {string} ySrc Index to Y coordinates in the response.
 * @param {list} xWrap A list of two values [lower_bound, upper_bound) to
 *     wrap-around X coordinates.
 * @param {list} yWrap A list of two values [lower_bound, upper_bound) to
 *     wrap-around Y coordinates.
 *
 * @return {hashmap} A hashmap with the following fields:
 *     'x': A list of X coordinate values.
 *     'y': A list of Y coordinate values.
 */
function getXYCoordLists(resp, xSrc, ySrc, xWrap, yWrap) {
  var xCoords = nestedIndex(resp, xSrc);
  var yCoords = nestedIndex(resp, ySrc);
  if (xCoords == null || yCoords == null) {
    return null;
  }
  if (!Array.isArray(xCoords)) {
    // Convert to array if it is a scalar.
    xCoords = [xCoords];
  }
  if (!Array.isArray(yCoords)) {
    // Convert to array if it is a scalar.
    yCoords = [yCoords];
  }
  if (xWrap) {
    for (var i = 0; i < xCoords.length; i++) {
      xCoords[i] = wrap(xCoords[i], xWrap);
    }
  }
  if (yWrap) {
    for (var i = 0; i < yCoords.length; i++) {
      yCoords[i] = wrap(yCoords[i], yWrap);
    }
  }
  return {
      'x': xCoords,
      'y': yCoords,
  };
}


/**
 * Obtain coordinates that come as dictionaries with the same set of keys.
 *
 * @param {hashmap} resp The response object.
 * @param {string} xSrc Index to X coordinates in the response.
 * @param {string} ySrc Index to Y coordinates in the response.
 * @param {list} xWrap A list of two values [lower_bound, upper_bound) to
 *     wrap-around X coordinates.
 * @param {list} yWrap A list of two values [lower_bound, upper_bound) to
 *     wrap-around Y coordinates.
 *
 * @return {hashmap} A hashmap with the following fields:
 *     keys: A sorted list of keys.
 *     originalX: The dictionary of original values.
 *     x: A dictionary of X coordinate values.
 *     y: A dictionary of Y coordinate values.
 */
function getXYCoordMaps(resp, xSrc, ySrc, xWrap, yWrap) {
  var xCoords = nestedIndex(resp, xSrc);
  var yCoords = nestedIndex(resp, ySrc);
  if (xCoords == null || yCoords == null) {
    return null;
  }
  var originalXCoords = jQuery.extend({}, xCoords);
  var keys = Object.keys(xCoords);
  keys.sort();

  if (xWrap) {
    for (var key in xCoords) {
      if (xCoords.hasOwnProperty(key)) {
        xCoords[key] = wrap(xCoords[key], xWrap);
      }
    }
  }
  if (yWrap) {
    for (var key in yCoords) {
      if (yCoords.hasOwnProperty(key)) {
        yCoords[key] = wrap(yCoords[key], yWrap);
      }
    }
  }

  return {
      'keys': keys,
      'originalX': originalXCoords,
      'x': xCoords,
      'y': yCoords,
  };
}


/**
 * Obtain a dict of values and wrap them if requested.
 *
 * @param {hashmap} resp The response object.
 * @param {string} src Index to the data in the response.
 * @param {list} wrap A list, [lower_bound, upper_bound], to wrap-around values.
 *
 * @return {hashmap} A hashmap with the following fields:
 *     keys: A sorted list of keys.
 *     original: The dictionary of original values.
 *     values: The processed values.
 */
function getCoordMap(resp, src, wrap) {
  var values = nestedIndex(resp, src);
  if (values == null) {
    return null;
  }
  var keys = Object.keys(values);
  keys.sort();
  var originalValues = jQuery.extend({}, values);

  if (wrap) {
    for (var key in values) {
      if (values.hasOwnProperty(key)) {
        values[key] = wrap(values[key], wrap);
      }
    }
  }

  return {
    'keys': keys,
    'original': originalValues,
    'values': values,
  };
}


/**
 * Obtain a list of values and wrap them if requested.
 *
 * @param {hashmap} resp The response object.
 * @param {string} src Index to the data in the response.
 * @param {list} wrap A list, [lower_bound, upper_bound], to wrap-around values.
 *
 * @return {hashmap} A hashmap that has the following fields:
 *     original: The list of original values.
 *     values: The processed values.
 */
function getCoordList(resp, src, wrap) {
  var values = nestedIndex(resp, src);
  if (values == null) {
    return null;
  }
  if (!Array.isArray(values)) {
    // Convert to array if it is a scalar.
    values = [values];
  }
  var originalValues = jQuery.extend([], values);

  if (wrap) {
    for (var i = 0; i < values.length; i++) {
      values[i] = wrap(values[i], wrap);
    }
  }
  return {
    'original': originalValues,
    'values': values,
  };
}


/**
 * Obtain coordinates as a list of dictionaries, where each has X/Y coordinates.
 *
 * The coordinates are wrapped if requested.
 *
 * @param {hashmap} resp The response object.
 * @param {string} seriesSrc Index to the data series in the response.
 * @param {list} xWrap A list of two values [lower_bound, upper_bound) to
 *     wrap-around X coordinates.
 * @param {list} yWrap A list of two values [lower_bound, upper_bound) to
 *     wrap-around Y coordinates.
 *
 * @return {hashmap} values A dictionary of data series in the form of
 *     {key: {'x': [], 'y': []}}, each has a list of X coordinates and a list of
 *     Y coordinates.
 */
function getMapOfCoordLists(resp, seriesSrc, xWrap, yWrap) {
  var values = nestedIndex(resp, seriesSrc);
  if (values == null) {
    return null;
  }
  var keys = Object.keys(values);
  keys.sort();

  if (xWrap) {
    for (var key in values) {
      if (values.hasOwnProperty(key)) {
        values[key]['x'] = wrap(values[key]['x'], xWrap);
      }
    }
  }
  if (yWrap) {
    for (var key in values) {
      if (values.hasOwnProperty(key)) {
        values[key]['y'] = wrap(values[key]['y'], yWrap);
      }
    }
  }
  return values;
}
