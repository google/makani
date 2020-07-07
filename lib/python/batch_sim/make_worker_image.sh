#!/bin/bash
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
# Creates a new image for batch sim workers.
#
# The image is created by bringing up a GCE instance, which configures itself
# into the new image by running a startup script.  The instance is then deleted,
# its disk is converted to an image, and finally the disk is deleted.
#
# Coordination with the GCE instance is performed via its 'custom_status'
# metadata field.  (GCE has its own 'status' field; we wouldn't actually collide
# with it, but we'd create opportunities for confusion.)
#
# This script can easily be made generic up to the instance's startup script.

source "${MAKANI_HOME}/lib/scripts/mbash.sh"

# Source shflags.
source /opt/shflags-1.0.3/src/shflags

DEFINE_string 'image_name' '' 'Name of the image to create.'
DEFINE_string 'instance_name' 'batch-sim-worker-image-maker' \
  'Name of the image-generating GCE instance.'
DEFINE_string 'instance_zone' 'us-central1-f' \
  'Zone in which to run the image-generating GCE instance.'
DEFINE_boolean 'overwrite' false 'Allow overwriting an existing image.'

FLAGS "$@" || exit $?
eval set -- "${FLAGS_ARGV}"

# Exit if any command returns an error code.
set -o errexit

# Check preconditions.
if [[ -z "${FLAGS_image_name}" ]]; then
  echo '--image_name must be specified.'
  exit 1
fi

if gcloud compute images list \
    | grep -q -E "\b${FLAGS_image_name}[^a-z0-9\-]"; then
  if [[ "${FLAGS_overwrite}" -eq "${FLAGS_FALSE}" ]]; then
    echo "An image with name ${IMAGE_NAME} already exists.  Either re-run with"
    echo 'a different --image_name, or specify --overwrite.'
    exit 1
  fi
fi

readonly IMAGE_NAME="${FLAGS_image_name}"
readonly ZONE="${FLAGS_instance_zone}"
readonly INSTANCE_NAME="${FLAGS_instance_name}"

readonly IMAGE_FAMILY="debian-9"
readonly IMAGE_PROJECT="debian-cloud"
readonly STARTUP_SCRIPT="${MAKANI_HOME}/lib/python/batch_sim/make_worker_image_startup_stretch.sh"

# Create the instance whose disk will generate the new image.
gcloud compute instances create "${INSTANCE_NAME}" \
  --zone="${ZONE}" \
  --image-family="${IMAGE_FAMILY}" \
  --image-project="${IMAGE_PROJECT}" \
  --machine-type=n1-standard-1 \
  --scopes compute-rw \
  --metadata custom_status=WORKING \
  --metadata-from-file startup-script="${STARTUP_SCRIPT}"


# Wait for image to terminate.
echo 'Waiting for instance to finish. (This will take a few minutes.)'
time_waited=0
while [[ "${time_waited}" -lt 600 ]]; do
  status="$(gcloud compute instances describe ${INSTANCE_NAME} --zone=${ZONE} \
    | grep -A 1 'key: custom_status' \
    | awk '/value/ {print $2}')"
  if [[ "${status}" != 'WORKING' ]]; then
    break
  fi
  echo 'Instance is still working.  Waiting 15 seconds.'
  time_waited="$(expr ${time_waited} + 15)"
  sleep 15
done

# Exit on non-SUCCESS status.
if [[ "${status}" == 'ERROR' ]]; then
  echo 'Instance terminated with status ERROR.  Inspect startup script logs'
  echo 'for more information.'
  exit 1
elif [[ "${status}" == 'WORKING' ]]; then
  echo 'Instance failed to complete soon enough.  Inspect startup script logs'
  echo 'for more information.'
  exit 1
elif [[ "${status}" != 'SUCCESS' ]]; then
  echo "Instance has unrecognized status ${status}."
  exit 1
fi

# Delete the instance.
gcloud compute instances delete "${INSTANCE_NAME}" \
  --zone="${ZONE}" \
  --keep-disks boot \
  --quiet

# Clear the old image if needed.  We've already confirmed that --overwrite
# was specified if it was needed.
if gcloud compute images list | grep -q "${IMAGE_NAME}"; then
  gcloud compute images delete "${IMAGE_NAME}" --quiet
fi

# Create the new image.
gcloud compute images create "${IMAGE_NAME}" \
  --source-disk="${INSTANCE_NAME}" \
  --source-disk-zone="${ZONE}"

# Delete the persistent disk from which the image was created.
gcloud compute disks delete "${INSTANCE_NAME}" --zone="${ZONE}" --quiet
