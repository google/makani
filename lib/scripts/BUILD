package(default_visibility = ["//visibility:public"])

exports_files(["network_config.sh"])

sh_library(
    name = "mbash",
    srcs = [
        "mbash.sh",
    ],
)

sh_binary(
    name = "sim_tcpdump",
    srcs = ["sim_tcpdump.sh"],
    data = [
        "network_config.sh",
        ":mbash",
    ],
)

filegroup(
    name = "all_scripts",
    # To filter out text editor backup files, restrict to files whose
    # basename has an alphabetical first character and an alphanumeric
    # last character.
    srcs = [
        f
        for f in glob([
            "*.sh",
            "developer/*",
            "operator/*",
        ])
        if f.split("/")[-1][0].isalpha() and f[-1].isalnum()
    ],
)
