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

var allColors = [
  '#74add1', '#abd9e9', '#e0f3f8', '#ffffbf', '#fee090', '#fdae61', '#f46d43',
  '#d73027', '#a50026'
];

/**
 * Sets the color and link text of a table cell based on the input value.
 *
 * @param {td element} tableCell The td element to update.
 * @param {number} value The value used for color/text selection.
 */
function SetColorAndLinkText(tableCell, value) {
  var color = '#888888';
  var text = 'nan';
  if (!isNaN(value)) {
    var idx = Math.round(value * allColors.length);
    color = allColors[Math.min(Math.max(0, idx), allColors.length - 1)];
    text = Math.round(100.0 * value).toString();
  }
  $(tableCell).attr('style', 'background-color: ' + color);
  $(tableCell).find('a').text(text);
}

/**
 * Updates table cells based on score-selection checkboxes.
 *
 * @param {jQuery} checkboxes Collection of score-selector checkbox elements.
 */
function UpdateTableCellsFromCheckboxes(checkboxes) {
  var activeScoreIndices = [];

  console.log(checkboxes);

  checkboxes.each(function(i) {
    if (this.checked) {
      activeScoreIndices.push(i);
    }
  });

  // Each element of class "value" with the "data-scores" attribute represents
  // the output of a simulation. Update the color and link text based on the
  // currently-active scores.
  $('.value')
      .each(function() {
        var scores = $(this).data().scores;
        if (scores) {
          if (scores.length == 0) {
            SetColorAndLinkText(this, NaN);
          } else {
            var maxScore = 0.0;
            for (var i = 0; i < activeScoreIndices.length; i++) {
              var score = scores[activeScoreIndices[i]];
              if (score == 'NaN') score = NaN;
              maxScore = Math.max(maxScore, score);
            }

            // Max out the displayed score at "999".
            maxScore = Math.min(maxScore, 9.99);

            SetColorAndLinkText(this, maxScore);
          }
        }
      });
}
