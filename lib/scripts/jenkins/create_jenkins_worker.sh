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
# Creates a Jenkins worker instance.

if [[ $# < 2 ]]; then
  echo 'Usage:'
  echo '    ./create_jenkins_worker.sh <worker_name> <disk_image> [extra flags]'
  exit 1
fi

WORKER_NAME="$1"
DISK_IMAGE="$2"
shift 2

gcloud compute instances create "${WORKER_NAME}" $@ \
    --image "${DISK_IMAGE}" \
    --zone us-central1-a \
    --machine-type=n1-highmem-4 \
    --local-ssd=interface=NVME \
    --service-account='log-analyzer@makani.google.com.iam.gserviceaccount.com' \
    --scopes 'https://www.googleapis.com/auth/gerritcodereview,https://www.googleapis.com/auth/devstorage.read_only' \
    --metadata startup-script='#! /bin/bash
      DEV="/dev/nvme0n1"
      MNT="/home/jenkins/.cache/bazel"
      sudo mkfs.ext4 -F "${DEV}"
      sudo mkdir -p "${MNT}"
      sudo mount "${DEV}" "${MNT}"
      sudo chown jenkins:jenkins "/home/jenkins/.cache"
      sudo chown jenkins:jenkins "${MNT}"'
