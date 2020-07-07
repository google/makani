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

/* Utilities to maintain a history buffer.
 *
 * These utilities are not being used as of 2015-05-18. We expect them to be
 * used soon in upcoming visualization widgets.
 */


/** @constructor A class to manage a history buffer
 *
 * @param {integer} desiredHistory Number of elements in the buffer.
 */
function HistoryBuffer(desiredHistory) {
  // Maximum length of the history in time.
  this.desiredHistory = desiredHistory;
  // Time of occurrence for each point in the history.
  this.historicTime = [];
  // Records for each point in the history.
  this.historicRecords = [];
  // The current number of records in the history.
  this.length = 0;
}


/**
 * Discards expired records from the history buffer.
 * The kept history has the range of
 * (currentTimestamp - desiredHistory, currentTimestamp].
 *
 * @param {float} currentTimestamp The current timestamp.
 * @param {function} callback An optional callback function invoked when a
 *     record is retired. It takes the form of callback(record, timestamp).
 */
HistoryBuffer.prototype.shift = function(currentTimestamp, callback) {
  var earliestTimestamp = currentTimestamp - this.desiredHistory;
  while (this.historicTime.length > 0 &&
         this.historicTime[0] <= earliestTimestamp) {
    if (callback) {
      callback(this.historicRecords[0], this.historicTime[0]);
    }
    this.historicTime.shift();
    this.historicRecords.shift();
    this.length -= 1;
  }
};


/**
 * Add new records to the history buffer.
 *
 * Only records newer than our newest record can be added.
 *
 * @param {object} newRecord A new object to add.
 * @param {float} timestamp A timestamp associated with the new object.
 * @param {function} retireCallback A callback to be called when retiring an
 *     old record.
 */
HistoryBuffer.prototype.push = function(newRecord, timestamp,
                                        retireCallback) {
  if (this.length && this.historicTime[this.length - 1] > timestamp)
    return;
  this.historicTime.push(timestamp);
  this.historicRecords.push(newRecord);
  this.length += 1;
  this.shift(timestamp, retireCallback);
};


/** Map and return the historic records.
 *
 * @param {function} callback The callback to be applied to each record that
 *     returns the value as part of the returned data list.
 * @return {list} A list of data values each associated with a record in
 *     history.
 */
HistoryBuffer.prototype.map = function(callback) {
  return this.historicRecords.map(callback);
};


/**
 * Extract a single value in the history buffer.
 *
 * @param {int} index Index to the record to extract. 0 corresponds to the
 *     earliest record.
 * @return {object} The indexed record.
 */
HistoryBuffer.prototype.get = function(index) {
  return this.historicRecords[index];
};


/**
 * @constructor A list of history buffers.
 *
 * @param {integer} desiredHistory Number of elements in the buffer.
 */
function HistoryBufferList(desiredHistory) {
  this.histories = [];
  this.desiredHistory = desiredHistory;
}


/** Get the history buffer record at a given index. Create if none exists.
 *
 * @param {int} index Index of the history buffer to retrieve.
 * @return {HistoryBuffer} A HistoryBuffer object.
 */
HistoryBufferList.prototype.get = function(index) {
  while (this.histories.length <= index) {
    var history = new HistoryBuffer(this.desiredHistory);
    this.histories.push(history);
  }
  return this.histories[index];
};


/** The number of history buffers in the list.
 *
 * @return {int} Number of history buffers.
 */
HistoryBufferList.prototype.length = function() {
  return this.histories.length;
};
