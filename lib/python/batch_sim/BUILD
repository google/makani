package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:misc_rules.bzl", "gcs_fetch")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "gcloud_util",
    srcs = [
        "gcloud_constants.py",
        "gcloud_util.py",
    ],
    deps = [
        requirement("fasteners"),
        requirement("google-api-python-client"),
        requirement("httplib2"),
        requirement("monotonic"),
        requirement("oauth2client"),
        requirement("python-gflags"),
        requirement("uritemplate"),
    ],
)

py_test(
    name = "gcloud_util_test",
    srcs = ["gcloud_util_test.py"],
    data = [
        "testdata/compute-discovery.json",
        "testdata/storage-discovery.json",
    ],
    deps = [
        ":gcloud_util",
        "//lib/python:test_util",
    ],
)

py_library(
    name = "batch_sim",
    srcs = [
        "batch_sim_util.py",
        "client.py",
        "worker.py",
    ],
    data = [
        "worker_startup.sh",
        "//lib/scripts:sim_tcpdump",
    ],
    deps = [
        ":gcloud_util",
        "//lib/python:gsutil",
        "//lib/python:os_util",
        "//lib/python:wing_flag",
        "//lib/python/h5_utils",
    ],
)

py_library(
    name = "parameter_override",
    srcs = [
        "parameter_override.py",
    ],
    deps = [
        "//config:mconfig",
    ],
)

py_library(
    name = "parameter_sweeps",
    srcs = [
        "parameter_sweeps.py",
    ],
)

# TODO: Get jquery from the third_party repository once the
# webmonitor's direct symlink is no longer needed. See http://b/32087843.
gcs_fetch(
    name = "jquery.js",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/jquery.js",
    gunzip = False,
    sha256 = "0ba081f546084bd5097aa8a73c75931d5aa1fc4d6e846e53c21f98e6a1509988",
)

gcs_fetch(
    name = "d3.v4.min.js",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/d3.v4.min.js",
    gunzip = False,
    sha256 = "8585db4092b8a9d26201e0d58e343d1b40fa034c4b9c343878923d7649bb1699",
)

gcs_fetch(
    name = "c3.min.js",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/c3.min.js",
    gunzip = False,
    sha256 = "5406d18ddb3e57d0fb72299a6255c742e49f8780f43de0f383376fd707ab63ee",
)

gcs_fetch(
    name = "c3.css",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/c3.css",
    gunzip = False,
    sha256 = "f753bbae2a4bc73636853a142896bfe167bb46499c12afa526162e9f0c3d54b9",
)

gcs_fetch(
    name = "bokeh-0.13.0.min.css",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/bokeh-0.13.0.min.css",
    gunzip = False,
    sha256 = "a56c8294d962745109f0370e321c83aaad2d409b53f01567d1404247a8f9902c",
)

gcs_fetch(
    name = "bokeh-0.13.0.min.js",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/bokeh-0.13.0.min.js",
    gunzip = False,
    sha256 = "9256e69ba5365129f3b56074e4b4fe36e0600e94212df2516542fb8b26a14639",
)

py_library(
    name = "parameter_tables",
    srcs = [
        "parameter_tables.py",
    ],
    data = [
        "bokeh-0.13.0.min.css",
        "bokeh-0.13.0.min.js",
        "c3.css",
        "c3.min.js",
        "d3.v4.min.js",
        "frontend/shared_library.js",
        "frontend/sweeps_report.css",
        "frontend/sweeps_report.html",
        "frontend/sweeps_report.js",
        "jquery.js",
        "scoring_function_util.js",
    ],
    deps = [
        ":batch_sim",
        ":scoring_functions_plot",
        "//config:mconfig",
        "//control:h2py_system_types",
        "//lib/python:build_info",
        "//lib/python:dict_util",
        requirement("matplotlib"),
    ],
)

py_library(
    name = "scoring_functions",
    srcs = [
        "scoring_functions/__init__.py",
        "scoring_functions/aero.py",
        "scoring_functions/buoy.py",
        "scoring_functions/crosswind.py",
        "scoring_functions/gs02.py",
        "scoring_functions/hover.py",
        "scoring_functions/kinematics.py",
        "scoring_functions/loads.py",
        "scoring_functions/power.py",
        "scoring_functions/rotors.py",
        "scoring_functions/scoring_functions_util.py",
        "scoring_functions/software.py",
        "scoring_functions/status.py",
        "scoring_functions/tether.py",
        "scoring_functions/trans_in.py",
    ],
    data = [
        "//database:m600/m600_tail_aero_tables.json",
        "//database:m600/rotor_rev3_stability_tables.json",
        "//database:m600/rotor_rev4_stability_tables.json",
    ],
    deps = [
        ":batch_sim",
        ":flight_modes",
        "//analysis/aero:aero_ssam",
        "//analysis/aero:apparent_wind_util",
        "//analysis/aero:hover_model",
        "//analysis/control:geometry",
        "//analysis/log_analysis:loop_averager",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/network:h2py_aio_labels",
        "//control:h2py_control_types",
        "//control:h2py_system_types",
        "//lib/python:c_helpers",
        "//lib/python/h5_utils:numpy_utils",
        "//system:h2py_labels",
        requirement("numpy"),
        requirement("pandas"),
        requirement("scipy"),
    ],
)

py_library(
    name = "scoring_functions_worker",
    srcs = [
        "scoring_functions/worker.py",
    ],
    deps = [
        ":scoring_functions",
        "//analysis/checks:autocheck",
        "//analysis/crosswind_batch_sims:crosswind_sweep_checks",
        "//lib/python:json_util",
        "//lib/python:struct_tree",
    ],
)

py_library(
    name = "scoring_functions_plot",
    srcs = ["scoring_functions/plot.py"],
    deps = [
        requirement("bokeh"),
        requirement("Jinja2"),
        requirement("packaging"),
    ],
)

py_test(
    name = "scoring_functions_test",
    srcs = ["scoring_functions/scoring_functions_test.py"],
    main = "scoring_functions/scoring_functions_test.py",
    deps = [
        ":scoring_functions",
    ],
)

py_test(
    name = "aero_scoring_functions_test",
    srcs = ["scoring_functions/aero_test.py"],
    main = "scoring_functions/aero_test.py",
    deps = [
        ":scoring_functions",
    ],
)

py_library(
    name = "flight_modes",
    srcs = [
        "flight_modes.py",
    ],
)

py_test(
    name = "flight_modes_test",
    srcs = ["flight_modes_test.py"],
    deps = [
        ":flight_modes",
        "//control:h2py_control_types",
        "//lib/python:c_helpers",
    ],
)

filegroup(
    name = "overrides_table_style",
    srcs = ["overrides_table_style.css"],
)

py_library(
    name = "test_util",
    srcs = [
        "gcloud_fakes.py",
        "testing_fakes.py",
    ],
    deps = [
        "//lib/python:os_util",
        "//lib/python:test_util",
    ],
)

py_test(
    name = "batch_sim_test",
    srcs = ["batch_sim_test.py"],
    data = ["//lib/pcap_to_hdf5:empty_log.h5"],
    deps = [
        ":batch_sim",
        ":test_util",
    ],
)

filegroup(
    name = "batch_sim_comparison_resources",
    data = [
        "d3.v4.min.js",
        "frontend/comparison_report.html",
        "frontend/comparison_report.js",
        "frontend/shared_library.js",
        "frontend/sweeps_report.css",
    ],
)
