========== OVERVIEW ===========

The log synchronizer automatically checks for newly generated logs and
uploads them to the cloud.

The log synchronizer scans the specified directory recursively, detects HDF5
logs that do not exist in the cloud, and automatically uploads them there.
It can run as a cron job, or be invoked manually by user.

Some design features for the users to be aware of:

1. Centralize configuration to ease management and keep things on track.

   a. A group of logs is stored as a "collection" in a cloud path.

   b. Each collection can have multiple "sources", each source is a combination
      of source machine's hostname and the local directory on that machine that
      contains the logs to upload.

   c. There is a centralized configuration file about where logs come from and
      end up at. This file resides in the cloud and is visible to all machines.
      By default, its location is gs://m600_testing_logs/configs.json.

2. Background mode: the synchronizer tries not to disturb/affect active jobs.
   It repeatedly checks and stops uploading when at least one of the following
   occurs:

   a. Internet is unavailable,

   b. Cpu or memory is being actively used,

   c. Some high priority tasks are running.

========== INSTALLATION AND USAGE ===========

1. To install necessary packages, go to lib/scripts/install/ and run
   ./install_packages.sh

2. To configure for your machine, do the following (this only needs to happen
   once):

   a. To specify how the log synchronizer should behave on the local machine,
      several flags are avaiable. Their default values are:
      --preserve_local
      --max_cpu_percent=30
      --max_mem_percent=80
      --exclusive_binaries=sim,recorder,vis

   b. To modify the global behavior about where logs come from or end up at,
      edit the local copy of the cloud configuration at
      configs/cloud_config.cfg. This file is kept track by git. Commit any
      change and send code review to anyone listed at configs/MODERATORS.
      Once submitted, publish the configuration by running

      publish_cloud_config.py

      The moderator should confirm that the configuration is published.

      *** ATTN ***
      The cloud_config.json supports CPP-style comments.
      When adding a new source, please add a comment about who owns the source.

3. Manually invoke the log synchronizer:

   $MAKANI_HOME/lib/scripts/operator/run_logsync.sh

4. To deploy the synchronizer for automatic uploading:

   $MAKANI_HOME/lib/scripts/set_locsync_cron.sh

   The cron job will produce its own logs at $MAKANI_HOME/logs/logsync.log

5. To manually upload a directory or file:
   python $MAKANI_HOME/lib/log_synchronizer/manual_upload.py
       --source_path <path_to_file_or_dir_to_upload>
       --destination_path gs://<bucket>/<path_to_parent_dir>
       --[no]preserve_local
       [--source_pattern=<regular_expression_pattern>]

