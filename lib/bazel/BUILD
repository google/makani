# This file specifies:
#  - A cc_toolchain for each architecture we compile for. Each
#    cc_toolchain links to a toolchain definition in the CROSSTOOL file via its
#    "cpu" attribute.
#  - A config_setting for each non-local architecture. These allow rules to
#    be tailored to specific architectures via select statements.
#  - pyembed, for non-Bazel Python executables with Bazel-built Python
#    dependencies.
# Most fields are copy/pasted from tools/cpp/BUILD in the Bazel repository.

package(default_visibility = ["//visibility:public"])

load("@bazel_tools//tools/python:toolchain.bzl", "py_runtime_pair")
load("@python_pip//:requirements.bzl", "all_requirements", "requirement")
load("@rules_cc//cc:defs.bzl", "cc_toolchain", "cc_toolchain_suite")
load("@rules_python//python:defs.bzl", bazel_py_library = "py_library")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library")
load(":cc_toolchain_config.bzl", "cc_toolchain_config")

filegroup(
    name = "empty",
    srcs = [],
)

filegroup(
    name = "tms570_files",
    srcs = [
        "tms570/ar",
        "tms570/gcc",
        "//avionics/firmware/startup:linker_scripts",
        "//third_party_toolchains/gcc_arm_none_eabi:everything",
    ],
)

filegroup(
    name = "q7_all_files",
    srcs = [
        "//lib/bazel/q7:tool-wrappers",
        "//third_party_toolchains/arm_linux_gnueabi:everything",
    ],
)

cc_toolchain_suite(
    name = "toolchain",
    toolchains = {
        "k8": ":cc-compiler-local",
        "k8|clang": ":cc-compiler-k8",
        "q7": ":cc-compiler-q7",
        "tms570": ":cc-compiler-tms570",
    },
)

cc_toolchain(
    name = "cc-compiler-local",
    all_files = ":empty",
    compiler_files = ":empty",
    dwp_files = ":empty",
    dynamic_runtime_lib = ":empty",
    linker_files = ":empty",
    objcopy_files = ":empty",
    static_runtime_lib = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    toolchain_config = ":cc_toolchain_config_local",
)

cc_toolchain_config(
    name = "cc_toolchain_config_local",
    compiler = "gcc",
    cpu = "k8",
)

cc_toolchain(
    name = "cc-compiler-k8",
    all_files = ":empty",
    compiler_files = ":empty",
    dwp_files = ":empty",
    dynamic_runtime_lib = ":empty",
    linker_files = ":empty",
    objcopy_files = ":empty",
    static_runtime_lib = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    toolchain_config = ":cc_toolchain_config_k8",
)

cc_toolchain_config(
    name = "cc_toolchain_config_k8",
    compiler = "clang",
    cpu = "k8",
)

cc_toolchain(
    name = "cc-compiler-q7",
    all_files = ":q7_all_files",
    compiler_files = ":q7_all_files",
    dwp_files = ":empty",
    dynamic_runtime_lib = ":empty",
    linker_files = ":q7_all_files",
    objcopy_files = "//lib/bazel/q7:objcopy",
    static_runtime_lib = ":empty",
    strip_files = "//lib/bazel/q7:strip",
    supports_param_files = 0,
    toolchain_config = ":cc_toolchain_config_q7",
)

cc_toolchain_config(
    name = "cc_toolchain_config_q7",
    cpu = "q7",
)

cc_toolchain(
    name = "cc-compiler-tms570",
    all_files = ":tms570_files",
    compiler_files = ":empty",
    dwp_files = ":empty",
    dynamic_runtime_lib = ":empty",
    linker_files = ":tms570_files",
    objcopy_files = ":empty",
    static_runtime_lib = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    toolchain_config = ":cc_toolchain_config_tms570",
)

cc_toolchain_config(
    name = "cc_toolchain_config_tms570",
    cpu = "tms570",
)

config_setting(
    name = "k8_gcc_mode",
    values = {
        "compiler": "gcc",
        "cpu": "k8",
    },
)

config_setting(
    name = "k8_clang_mode",
    values = {
        "compiler": "clang",
        "cpu": "k8",
    },
)

config_setting(
    name = "q7_mode",
    values = {"cpu": "q7"},
)

config_setting(
    name = "tms570_mode",
    values = {"cpu": "tms570"},
)

py_runtime(
    name = "python-2.7.16",
    interpreter_path = "/opt/makani/python2/bin/python",
    python_version = "PY2",
)

py_runtime(
    name = "python-3.6.7",
    interpreter_path = "/opt/makani/python3/bin/python",
    python_version = "PY3",
)

py_runtime_pair(
    name = "python_runtime_pair",
    py2_runtime = ":python-2.7.16",
    py3_runtime = ":python-3.6.7",
)

toolchain(
    name = "py_toolchain",
    toolchain = ":python_runtime_pair",
    toolchain_type = "@bazel_tools//tools/python:toolchain_type",
)

# pyembed is used to run a script that depends on Bazel-built Python
# dependencies.
#
# If you want access to a dependency that isn't included, add it to the deps of
# _pyembed_internal.
#
# Usage:
#     bazel build lib/bazel:pyembed
#     bazel-out/k8-fastbuild/bin/lib/bazel/pyembed <cmd>
# This will run <cmd> with MAKANI_HOME set to pyembed's runfiles directory,
# allowing Python modules present there to be imported normally.
# TODO: Clean up or remove pyembed.  It generally doesn't work as
# expected, since there's no way to access pip imported python libraries.
sh_binary(
    name = "pyembed",
    srcs = ["pyembed.sh"],
    # By including _pyembed_internal as a data dependency, all Python files on
    # which _pyembed_internal depends will be included in this target's runfiles
    # directory.
    data = [":_pyembed_internal_py2only"],
)

py_binary(
    name = "jupyter",
    srcs = ["jupyter.py"],
    python_version = "PY2",
    deps = [":_pyembed_internal_py2only"],
)

py_binary(
    name = "python2",
    srcs = ["python_interactive.py"],
    main = "python_interactive.py",
    python_version = "PY2",
    deps = [":_pyembed_internal_py2only"],
)

py_binary(
    name = "python3",
    srcs = ["python_interactive.py"],
    main = "python_interactive.py",
    python_version = "PY3",
    deps = [":_pyembed_internal"],
)

bazel_py_library(
    name = "_pyembed_internal",
    srcs_version = "PY2AND3",
    deps = all_requirements,
)

py_library(
    name = "_pyembed_internal_py2only",
    srcs_version = "PY2ONLY",
    deps = [
        ":_pyembed_internal",
        ":bazel_util",
        "//analysis/aero:airfoil",
        "//analysis/aero:hover_model",
        "//analysis/aero:load_database",
        "//analysis/aero:wake_model",
        "//analysis/control:catenary",
        "//analysis/control:crosswind",
        "//analysis/control:dynamics",
        "//analysis/control:flap_limits",
        "//analysis/control:geometry",
        "//analysis/control:simple_aero",
        "//analysis/control:trans_in",
        "//analysis/control/replay:estimator_helper",
        "//analysis/control/replay:h2py_estimator_replay",
        "//analysis/crosswind_batch_sims:crosswind_sweeps_batch_sim",
        "//analysis/plot:mplot",
        "//analysis/plot/python:pyplot",
        "//analysis/util/simdata_analysis:compare_simdata",
        "//analysis/util/simdata_analysis:data_import",
        "//analysis/util/simdata_analysis:interactive_analysis",
        "//analysis/util/simdata_analysis:risk",
        "//analysis/util/simdata_analysis:score_prob",
        "//analysis/util/simdata_analysis:simdata",
        "//avionics/bootloader/firmware:h2py_update_server",
        "//avionics/common:aio",
        "//avionics/common:h2py_actuator_types",
        "//avionics/common:h2py_aio_header",
        "//avionics/common:h2py_aio_version",
        "//avionics/common:h2py_build_info",
        "//avionics/common:h2py_coning_sculling",
        "//avionics/common:h2py_cvt",
        "//avionics/common:h2py_faults",
        "//avionics/common:h2py_network_config",
        "//avionics/common:h2py_novatel_types",
        "//avionics/common:h2py_pack_aio_header",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/common:h2py_pack_winch_messages",
        "//avionics/common:h2py_servo_types",
        "//avionics/common:h2py_winch_messages",
        "//avionics/firmware/identity:h2py_identity_types",
        "//avionics/motor/firmware:h2py_flags",
        "//avionics/motor/firmware:h2py_pack_logging",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:h2py_message_type",
        "//avionics/servo/firmware:h2py_r22_param",
        "//avionics/servo/firmware:h2py_r22_types",
        "//common/c_math:h2py_coord_trans",
        "//common/c_math:h2py_filter",
        "//common/c_math:h2py_geometry",
        "//config:mconfig",
        "//control:h2py_control_params",
        "//control:h2py_control_types",
        "//control:h2py_ground_frame",
        "//control:h2py_pack_control_telemetry",
        "//control:h2py_sensor_util",
        "//control:h2py_system_params",
        "//control:h2py_system_types",
        "//gs/monitor:h2py_monitor_types",
        "//lib/pcap_to_hdf5:h2py_pack_capture_info",
        "//lib/python:h5log_loader",
        "//lib/python:ipython_completer",
        "//lib/python/batch_sim:scoring_functions",
        "//lib/python/h5_utils:numpy_utils",
        "//sim:h2py_pack_sim_messages",
        "//sim:h2py_pack_sim_telemetry",
        "//sim:h2py_sim_types",
        "//sim/physics:h2py_wind_frame",
        "//sim/physics:pywrap_physics",
        "//system:h2py_labels",
    ],
)

py_binary(
    name = "check_test_rules",
    srcs = ["check_test_rules.py"],
    deps = [":bazel_util"],
)

py_binary(
    name = "gcs_fetch",
    srcs = ["gcs_fetch.py"],
    deps = [
        "//lib/python/batch_sim:gcloud_util",
        requirement("cachetools"),
        requirement("certifi"),
        requirement("chardet"),
        requirement("enum34"),
        requirement("google-cloud-storage"),
        requirement("google-api-core"),
        requirement("google-cloud-core"),
        requirement("google-auth"),
        requirement("google-resumable-media"),
        requirement("googleapis-common-protos"),
        requirement("grpc-google-iam-v1"),
        requirement("grpcio"),
        requirement("idna"),
        requirement("protobuf"),
        requirement("pyasn1"),
        requirement("pyasn1_modules"),
        requirement("python-gflags"),
        requirement("pytz"),
        requirement("futures"),
        requirement("requests"),
        requirement("rsa"),
        requirement("urllib3"),
    ],
)

py_library(
    name = "bazel_util",
    srcs = ["bazel_util.py"],
)
