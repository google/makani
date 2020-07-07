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

// BUTTON_OFF and BUTTON_ON are String
// Boolean(BUTTON_OFF) == false
// Boolean(BUTTON_ON) == true
const BUTTON_OFF = '';
const BUTTON_ON = '1';

// FLIGHT_MODE_BUTTON_IDENTIFIERS is a String array corresponding to the flight
// mode slider buttons in the sweeps_report.html page.
const FLIGHT_MODE_BUTTON_IDENTIFIERS = [
  '#payout_modes', '#fulllength_modes', '#transin_modes', '#crosswind_modes',
  '#transout_modes', '#transform_modes', '#reelin_modes'];
let activeFlightModes = ['kFlightModeHoverAscend', 'kFlightModeHoverPayOut',
                         'kFlightModeHoverFullLength', 'kFlightModeHoverAccel',
                         'kFlightModeTransIn', 'kFlightModeCrosswindNormal',
                         'kFlightModeCrosswindPrepTransOut',
                         'kFlightModeHoverReelIn',
                         'kFlightModeHoverTransOut',
                         'kFlightModeHoverTransformGsUp',
                         'kFlightModeHoverTransformGsDown',
                         'kFlightModeHoverPrepTransformGsUp',
                         'kFlightModeHoverPrepTransformGsDown'];

/**
 * Opens the sidebar.
 *
 * NOTE: d3 does not work well for resetting the onclick
 * handler, so we use the DOM instead for openSidebar and closeSidebar.
 */
function openSidebar() {
  let scoreContainer = document.getElementById('metric_configuration');
  scoreContainer.style.padding = '0rem 1rem 1rem 1rem';
  scoreContainer.style.width = 'auto';

  let width = scoreContainer.getBoundingClientRect().width;
  let pullTab = document.getElementById('configuration_tab');
  pullTab.style.marginLeft = width + 'px';
  pullTab.onclick = closeSidebar;
  document.getElementById('main_section').style.marginLeft = width + 'px';
}

/**
 * Closes the sidebar.
 */
function closeSidebar() {
  let scoreContainer = document.getElementById('metric_configuration');
  scoreContainer.style.width = '0';
  scoreContainer.style.padding = null;

  let pullTab = document.getElementById('configuration_tab');
  pullTab.style.marginLeft = 0;
  pullTab.onclick = openSidebar;
  document.getElementById('main_section').style.marginLeft = '0';
}

/**
 * Activate the manual selection indicator when an individual score checkbox is
 * selected or de-selected.
 */
function activateManualSelection() {
  d3.select('#manual_selection_legend').html('Manual selection: Active');
  d3.select('#manual_selection_button')
      .property('checked', true)
      .property('disabled', '');
  rescore();
}

/**
 * Deactivate the manual selection. Re-apply all the filters.
 */
function deactivateManualSelection() {
  d3.select('#manual_selection_legend').html('Manual selection: Inactive');
  d3.select('#manual_selection_button')
      .property('checked', false)
      .property('disabled', 'disabled');
}

/**
 * Select-unselect experimental scores.
 */
function toggleExperimentalScores() {
  deactivateManualSelection();
  d3.select('#active_scores').selectAll('input').each(function(d, i) {
    let systemLabels = d3.select(this).data()["0"].system_labels;
    if (Array.from(systemLabels).includes('experimental')) {
      d3.select(this).attr(
          'active_system', d3.select('#experimental_scores').node().checked);
    }
  });
  rescore();
}

/**
 * Updates scores that match the string of the active systems radio button, and
 * rescores the page.
 */
function updateSystemScores() {
  deactivateManualSelection();

  let activeSystem = d3.select('#systems')
                         .select('input[name = "systems"]:checked')
                         .node()
                         .value;
  d3.select('#active_scores').selectAll('input').each(function(d, i) {
    let systemLabels = d3.select(this).data()["0"].system_labels;
    if (Array.from(systemLabels).includes(activeSystem) ||
        activeSystem == 'all') {
      d3.select(this).attr('active_system', true);
    } else {
      d3.select(this).attr('active_system', false);
    }
    // The state of 'experimental_scores' overrides the previous selection.
    if (Array.from(systemLabels).includes('experimental')) {
      d3.select(this).attr(
          'active_system', d3.select('#experimental_scores').node().checked);
    }
  });
  rescore();
}


/**
 * Toggles scores that match the given flightMode string, and rescores the page.
 * @param {JavaScript Object} button The associated selector button.
 * @param {Array of string} flightModes List of flight modes.
 */
function toggleFlightModeScores(button, flightModes) {
  // Switch the value of the button.
  if (Boolean(button.value)) {
    button.value = BUTTON_OFF;
  } else {
    button.value = BUTTON_ON;
  }

  // Un-check the "manual selection" indicator.
  deactivateManualSelection();


  // Update the list of active flight modes.
  for (i in flightModes) {
    if (Boolean(button.value)) {
      activeFlightModes.push(flightModes[i]);
    } else {
      let ind = activeFlightModes.indexOf(flightModes[i]);
      if (ind != -1) {
        activeFlightModes.splice(ind, 1);
      }
    }
  }

  // Update the active_flight_mode attribute of each score.
  d3.select('#active_scores').selectAll('input').each(function(d, i) {
    let systemLabels = d3.select(this).data()["0"].system_labels;

    if (systemLabels.findIndex(
        element => element.includes('kFlightMode')) == -1) {
      // If no flight mode label is present in systemLabels, the score spans
      // all the flight modes. Set active_flight_mode to true iff at least one
      // flight mode filter is ON.
      if (activeFlightModes.length != 0) {
        d3.select(this).attr('active_flight_mode', true);
      } else {
        d3.select(this).attr('active_flight_mode', false);
      }
    } else {
      // Else set active_flight_mode to true iff the flight mode labels
      // intersects with activeFlightModes.
      let flight_modes_intersect = activeFlightModes.filter(
          value => -1 !== systemLabels.indexOf(value));
      if (!flight_modes_intersect.length) {
        d3.select(this).attr('active_flight_mode', false);
      } else {
        d3.select(this).attr('active_flight_mode', true);
      }
    }

    // The state of 'experimental_scores' overrides the previous selection.
    if (Array.from(systemLabels).includes('experimental')) {
      d3.select(this).attr(
          'active_system', d3.select('#experimental_scores').node().checked);
    }
  });
  rescore();
}

/**
 * Selects all scores via checkboxes, and rescores the page.
 */
function selectAllScores() {
  deactivateManualSelection();
  d3.select('#experimental_scores').property('checked', true);
  d3.select('#active_scores')
      .selectAll('input')
      .property('checked', true)
      .attr('active_flight_mode', true)
      .attr('active_system', true);

  for (i in FLIGHT_MODE_BUTTON_IDENTIFIERS) {
    d3.select(FLIGHT_MODE_BUTTON_IDENTIFIERS[i])
        .property('checked', true)
        .attr('value', BUTTON_ON);
  }
  activeFlightModes = [
      'kFlightModeHoverAscend', 'kFlightModeHoverPayOut',
      'kFlightModeHoverFullLength', 'kFlightModeHoverAccel',
      'kFlightModeTransIn', 'kFlightModeCrosswindNormal',
      'kFlightModeCrosswindPrepTransOut', 'kFlightModeHoverReelIn',
      'kFlightModeHoverTransOut', 'kFlightModeHoverTransformGsUp',
      'kFlightModeHoverTransformGsDown', 'kFlightModeHoverPrepTransformGsUp',
      'kFlightModeHoverPrepTransformGsDown'];

  d3.select('#all_systems').property('checked', true);
  rescore();
}

/**
 * Clears all scores via checkboxes, and rescores the page.
 */
function clearAllScores() {
  deactivateManualSelection();
  d3.select('#experimental_scores').property('checked', false);
  d3.select('#active_scores')
      .selectAll('input')
      .property('checked', false)
      .attr('active_flight_mode', false);

  for (i in FLIGHT_MODE_BUTTON_IDENTIFIERS) {
    d3.select(FLIGHT_MODE_BUTTON_IDENTIFIERS[i])
        .property('checked', false)
        .attr('value', BUTTON_OFF);
  }
  activeFlightModes = [];

  rescore();
}

/**
 * Makes the score-selection checkboxes.
 * @param {string} element ID of the containing element.
 * @param {Array<dict>} metrics List of metrics.
 */
function makeCheckboxes(element, metrics) {
  d3.select('#experimental_scores').property('checked', false);
  let labels =
      d3.select(element).selectAll('input').data(metrics).enter().append(
          'label');
  labels.append('input')
      .attr('type', 'checkbox')
      .property(
          'checked',
          function(d) {
            if (d['system_labels'].includes('experimental')) {
              return false;
            } else {
              return true;
            }
          })
      .attr('name', function(d, i) { return 'score' + i; })
      .attr('value', function(d, i) { return 'score' + i; })
      .attr('active_flight_mode', true)
      .attr(
          'active_system',
          function(d) {
            if (d['system_labels'].includes('experimental')) {
              return d3.select('#experimental_scores').node().checked;
            } else {
              return true;
            }
          })
      .on('change', activateManualSelection);
  labels.append('text').text(function(d) { return d['name']; });
  labels.append('br');
}
