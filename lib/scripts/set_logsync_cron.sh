#!/bin/bash -eu
# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Add the log synchronizer as a cron job.

CRONTAB_PATH='/etc/crontab'

if grep -q 'run_logsync\.sh' "${CRONTAB_PATH}"; then
  echo "The cron job for the log synchronizer is already set up.";
else
  RUN_LOGSYNC_PATH="${MAKANI_HOME}/lib/scripts/operator/run_logsync.sh"
  LOGSYNC_LOG="${MAKANI_HOME}/logs/logsync.log"
  # Interval is in minutes.
  LOGSYNC_INTERVAL=48
  CRON_CMD="MAKANI_HOME=${MAKANI_HOME} ${RUN_LOGSYNC_PATH} > ${LOGSYNC_LOG} 2>&1"
  LOGSYNC_CRON_CONFIG="*/${LOGSYNC_INTERVAL} * * * * ${USER} ${CRON_CMD}"
  if echo "${LOGSYNC_CRON_CONFIG}" | sudo tee -a "${CRONTAB_PATH}" > /dev/null;
  then
    echo "The cron job for the log synchronizer is set up as";
    echo "${LOGSYNC_CRON_CONFIG}"
  else
    echo "Failed to set up the cron job for the log synchronizer."
    echo "Try the following steps to manually add the cron job:"
    echo "1. sudo vim ${CRONTAB_PATH}"
    echo "2. Append the following line to the end of the file:"
    echo "${LOGSYNC_CRON_CONFIG}"
    echo "3. Type \":wq\" to save and exit the file."
  fi
fi

