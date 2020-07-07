package(default_visibility = ["//visibility:public"])

load("//lib/bazel:misc_rules.bzl", "gcs_fetch")

filegroup(
    name = "turbsim",
    srcs = [
        "20181011-165906-006-00_10mps_00shear.h5",
        "20190619-074534-012-00_10mps_00shear.h5",
    ],
)

gcs_fetch(
    name = "20181011-165906-006-00_10mps_00shear.h5",
    gcs_path = ("gs://makani_pub/turbsim_databases/" +
                "20181008-006-Initial_ParkerRanch_turbB_refht21m/h5_files/" +
                "20181011-165906-006-00_10mps_00shear.h5"),
    sha256 = "3543ae323fe60222059c88634f1a09a70999926252dba1c56fadd43e010c7cc5",
)

gcs_fetch(
    name = "20190619-074534-012-00_10mps_00shear.h5",
    gcs_path = ("gs://makani_pub/turbsim_databases/" +
                "20190619-012-Offshore_turb10_refht32m/h5_files/" +
                "20190619-074534-012-00_10mps_00shear.h5"),
    sha256 = "ab221f7a818f42ea45ded44656d16c03fec16578daacb4d27482e7cca9015719",
)
