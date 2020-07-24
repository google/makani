package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:misc_rules.bzl", "gcs_fetch")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

gcs_fetch(
    name = "test/data/Crosswind_sweeps_monte_carlo_nightly_1531/overview_data.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/analysis/util/simdata_analysis/test/data/Crosswind_sweeps_monte_carlo_nightly_1531/overview_data.json.gz",
    gunzip = True,
    sha256 = "766e70f40747dfaf7f6877e3993ff71540549ae881b43b3079d20e1266c73543",
)

gcs_fetch(
    name = "test/data/Crosswind_sweeps_monte_carlo_nightly_1524/overview_data.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/analysis/util/simdata_analysis/test/data/Crosswind_sweeps_monte_carlo_nightly_1524/overview_data.json.gz",
    gunzip = True,
    sha256 = "123fd56a85027fcb1f1e5adcbe81c91af636df3401315b0edd99d0afcda56feb",
)

gcs_fetch(
    name = "test/data/Crosswind_sweeps_monte_carlo_nightly_1556/overview_data.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/analysis/util/simdata_analysis/test/data/Crosswind_sweeps_monte_carlo_nightly_1556/overview_data.json.gz",
    gunzip = True,
    sha256 = "863c7874b3fe78683c696de1cb33843e5c4dfbfeda4db99cc8cbf62ed8f85324",
)

gcs_fetch(
    name = "test/data/manual_crosswind_sweeps_3859/overview_data.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/analysis/util/simdata_analysis/test/data/manual_crosswind_sweeps_3859/overview_data.json.gz",
    gunzip = True,
    sha256 = "ef4d9ee199ff77c59bdbb071aaaa4d64e9d47b934c9783a2ccfe03a3c843b04e",
)

gcs_fetch(
    name = "test/data/manual_crosswind_sweeps_3859/configs.tar.gz",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/gcs_fetch/analysis/util/simdata_analysis/test/data/manual_crosswind_sweeps_3859/configs.tar.gz",
    gunzip = False,
    sha256 = "04203875d0f24bae7eb41ad683bf250cc2d4d2021d76e58a9b6589c01dce0a42",
)

py_library(
    name = "bootstrap",
    srcs = ["bootstrap.py"],
    deps = [
        requirement("numpy"),
    ],
)

py_test(
    name = "bootstrap_test",
    srcs = ["test/bootstrap_test.py"],
    deps = [":bootstrap"],
)

py_library(
    name = "statistic",
    srcs = ["statistic.py"],
    deps = [
        ":utils",
        requirement("pandas"),
    ],
)

py_test(
    name = "statistic_test",
    srcs = ["test/statistic_test.py"],
    deps = [
        ":bootstrap",
        ":statistic",
    ],
)

py_library(
    name = "utils",
    srcs = ["utils.py"],
    deps = [
        requirement("matplotlib"),
        requirement("numpy"),
        requirement("scipy"),
    ],
)

py_library(
    name = "compare_simdata",
    srcs = ["compare_simdata.py"],
    deps = [
        ":risk",
        requirement("scipy"),
    ],
)

py_test(
    name = "compare_simdata_test",
    srcs = ["test/compare_simdata_test.py"],
    data = [
        "test/data/Crosswind_sweeps_monte_carlo_nightly_1524/overview_data.json",
        "test/data/Crosswind_sweeps_monte_carlo_nightly_1531/overview_data.json",
    ],
    deps = [
        ":compare_simdata",
        ":data_import",
        ":simdata",
    ],
)

py_library(
    name = "risk",
    srcs = ["risk.py"],
    deps = [
        requirement("numpy"),
        requirement("pandas"),
    ],
)

py_test(
    name = "risk_test",
    srcs = ["test/risk_test.py"],
    data = ["test/data/Crosswind_sweeps_monte_carlo_nightly_1531/overview_data.json"],
    deps = [
        ":data_import",
        ":risk",
        ":simdata",
    ],
)

py_library(
    name = "data_import",
    srcs = ["data_import.py"],
    deps = [
        "//lib/python:dict_util",
        requirement("numexpr"),
        requirement("numpy"),
        requirement("pandas"),
        requirement("python-dateutil"),
        requirement("tables"),
    ],
)

py_test(
    name = "data_import_test",
    srcs = ["test/data_import_test.py"],
    data = [
        "test/data/Crosswind_sweeps_monte_carlo_nightly_1531/overview_data.json",
        "test/data/manual_crosswind_sweeps_3859/configs.tar.gz",
        "test/data/manual_crosswind_sweeps_3859/overview_data.json",
    ],
    deps = [":data_import"],
)

py_library(
    name = "score_prob",
    srcs = ["score_prob.py"],
    deps = [
        ":data_import",
        ":simdata",
    ],
)

py_test(
    name = "score_prob_test",
    srcs = ["test/score_prob_test.py"],
    data = [
        "test/data/Crosswind_sweeps_monte_carlo_nightly_1531/overview_data.json",
        "test/data/Crosswind_sweeps_monte_carlo_nightly_1556/overview_data.json",
    ],
    deps = [
        ":data_import",
        ":score_prob",
        ":simdata",
    ],
)

py_library(
    name = "simdata",
    srcs = ["simdata.py"],
    deps = [
        ":vardata",
    ],
)

py_test(
    name = "simdata_test",
    srcs = ["test/simdata_test.py"],
    data = ["test/data/Crosswind_sweeps_monte_carlo_nightly_1531/overview_data.json"],
    deps = [
        ":data_import",
        ":simdata",
        ":vardata",
    ],
)

py_library(
    name = "vardata",
    srcs = ["vardata.py"],
    deps = [
        ":bootstrap",
        ":statistic",
        ":utils",
    ],
)

py_test(
    name = "vardata_test",
    srcs = ["test/vardata_test.py"],
    data = ["test/data/Crosswind_sweeps_monte_carlo_nightly_1531/overview_data.json"],
    deps = [
        ":data_import",
        ":simdata",
    ],
)

py_library(
    name = "interactive_analysis",
    srcs = ["interactive_analysis.py"],
    deps = [
        ":compare_simdata",
    ],
)
