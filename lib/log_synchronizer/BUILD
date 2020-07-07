package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_test")

py_binary(
    name = "manual_upload",
    srcs = [
        "auto_upload.py",
        "manual_upload.py",
    ],
    main = "manual_upload.py",
    deps = [
        "//lib/python:gsutil",
        "//lib/python/batch_sim:gcloud_util",
        requirement("httplib2"),
        requirement("psutil"),
        requirement("python-gflags"),
    ],
)

py_binary(
    name = "synchronizer",
    srcs = [
        "auto_upload.py",
        "synchronizer.py",
    ],
    data = ["logsync_config.json"],
    main = "auto_upload.py",
    deps = [
        "//lib/python:gsutil",
        "//lib/python/batch_sim:gcloud_util",
        requirement("httplib2"),
        requirement("psutil"),
        requirement("python-gflags"),
    ],
)

py_test(
    name = "manual_test",
    srcs = ["manual_test.py"],
    deps = [":manual_upload"],
)

py_test(
    name = "synchronizer_test",
    srcs = ["synchronizer_test.py"],
    data = glob(["testdata/**/*.*"]),
    deps = [
        ":synchronizer",
        "//lib/python:gsutil_fakes",
    ],
)
