package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary")

sh_binary(
    name = "webmonitor_launcher",
    srcs = ["webmonitor_launcher.sh"],
    data = [
        ":webmonitor",
        "//gs/monitor2/commands:list",
    ],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)

py_binary(
    name = "webmonitor",
    srcs = ["manage.py"],
    data = [
        "configs/plot_defs.json",
        "//third_party/javascript:js",
    ] + glob([
        "**/*.js",
        "**/*.css",
        "**/*.html",
        "**/*.jpg",
        "**/*.png",
    ]),
    main = "manage.py",
    deps = [
        "//avionics/network:h2py_message_type",
        "//gs/monitor2/apps/dashboard:webmonitor_dashboard",
        "//gs/monitor2/apps/layout:webmonitor_layout",
        "//gs/monitor2/apps/plugins:webmonitor_plugins",
        "//gs/monitor2/apps/receiver:webmonitor_receiver",
        "//gs/monitor2/project:webmonitor_settings",
        requirement("Django"),
        requirement("django-extensions"),
        requirement("expiringdict"),
    ],
)
