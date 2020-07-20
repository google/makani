package(default_visibility = ["//visibility:public"])

load("//lib/bazel:misc_rules.bzl", "gcs_fetch")

exports_files(["m600/rotor_rev3_stability_tables.json"])

exports_files(["m600/rotor_rev4_stability_tables.json"])

filegroup(
    name = "local_databases",
    srcs = glob(["**/*.json"]),
)

filegroup(
    name = "m600_avl_database",
    srcs = [
        "m600/m600_aswing_baseline.json",
        "m600/m600_aswing_baseline_zero_angular_rate.json",
        "m600/m600_low_tail_no_winglets.json",
        "m600/m600_low_tail_no_winglets_zero_angular_rate.json",
        "m600/m600_vsaero.json",
        "m600/m600_vsaero_zero_angular_rate.json",
        "m600/spoiler_offset_tables.json",
    ],
)

filegroup(
    name = "oktoberkite_avl_database",
    srcs = [
        "oktoberkite/big_m600_r05_aswing_baseline.json",
        "oktoberkite/big_m600_r05_aswing_baseline_zero_angular_rate.json",
    ],
)

filegroup(
    name = "rotor_databases",
    srcs = [
        "m600/rotor_rev1_corrected.json",
        "m600/rotor_rev1_trimmed_corrected.json",
        "m600/rotor_rev2_corrected.json",
        "m600/rotor_rev3_corrected.json",
        "m600/rotor_rev4.json",
        "m600/rotor_rev4_3d.json",
    ],
)

filegroup(
    name = "sn03_high_aoa_databases",
    # buildifier: leave-alone
    srcs = [
        "m600/hover_model_sn03_wind_1.json",
        "m600/hover_model_sn03_wind_1.6.json",
        "m600/hover_model_sn03_wind_2.5.json",
        "m600/hover_model_sn03_wind_4.json",
        "m600/hover_model_sn03_wind_6.3.json",
        "m600/hover_model_sn03_wind_10.json",
        "m600/hover_model_sn03_wind_10_no_wake_model.json",
        "m600/hover_model_sn03_wind_16.json",
        "m600/hover_model_sn03_wind_25.json",
        "m600/hover_model_sn03_wind_40.json",
        "m600/hover_model_sn03_wind_63.json",
    ],
)

gcs_fetch(
    name = "m600/rotor_rev4_3d.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/rotor_databases/20180702_gen4_3d_tables.json.gz",
    gunzip = True,
    sha256 = "e45d739cfb280bb8300a1aee5d17376f99650da0d01be865315a9bc81d47d340",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_1.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_1.json.gz",
    gunzip = True,
    sha256 = "176d99250f224dd985b955d5b9c4bb62db83894d28375e0f0adb7fbff63ff0c3",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_1.6.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_1.6.json.gz",
    gunzip = True,
    sha256 = "0ae8d07a61e015dd52b036dd804dc14087634ec219501de859f2ff6f8df1ab3b",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_2.5.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_2.5.json.gz",
    gunzip = True,
    sha256 = "90cb092c1833aeafe5f8ee646a1905a07af321696081729b7bf03a1f089f6700",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_4.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_4.json.gz",
    gunzip = True,
    sha256 = "35b63d18d15a9ba07cc1e713630fa4137fc8b777488cbab0b9da40c60b097d53",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_6.3.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_6.3.json.gz",
    gunzip = True,
    sha256 = "93aa9dc3b9aedf36ce34778f48596d54f08b65935b7e27f62ce6285b46722c3c",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_10.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_10.json.gz",
    gunzip = True,
    sha256 = "1bcb9c6e37e5e7092b07bccea1fbecc7623ddae8b6e4c95b3f21b2a2da113d21",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_16.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_16.json.gz",
    gunzip = True,
    sha256 = "e42f14bd03b5373c14e4348b30222acc750792068b2c06338fc8575aed10e6d8",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_25.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_25.json.gz",
    gunzip = True,
    sha256 = "3968f0d66045de39cbbe691df2ee6efeb4107f9118113ef01f10e04ad9792ad2",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_40.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_40.json.gz",
    gunzip = True,
    sha256 = "70c013980636da5be39b20cb1b00fbe9274a359c4950870ecb2f11f9f30a1418",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_63.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180531_hover_model_sn03_wind_63.json.gz",
    gunzip = True,
    sha256 = "804b33dfa25b4068629d30817257982f26938552f356730ef3f0035b8f810644",
)

gcs_fetch(
    name = "m600/hover_model_sn03_wind_10_no_wake_model.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/20180403_hover_model_sn03_wind_10_no_wake_model.json.gz",
    gunzip = True,
    sha256 = "b189aff1718474611141260ddd0514bcea506be276f4868aecfd742cde5f4a32",
)

filegroup(
    name = "oktoberkite_high_aoa_databases",
    # buildifier: leave-alone
    srcs = [
        "oktoberkite/hover_model_okt_wind_1.json",
        "oktoberkite/hover_model_okt_wind_1.6.json",
        "oktoberkite/hover_model_okt_wind_2.5.json",
        "oktoberkite/hover_model_okt_wind_4.json",
        "oktoberkite/hover_model_okt_wind_6.3.json",
        "oktoberkite/hover_model_okt_wind_10.json",
        "oktoberkite/hover_model_okt_wind_10_no_wake_model.json",
        "oktoberkite/hover_model_okt_wind_16.json",
        "oktoberkite/hover_model_okt_wind_25.json",
        "oktoberkite/hover_model_okt_wind_40.json",
        "oktoberkite/hover_model_okt_wind_63.json",
    ],
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_1.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_1.json.gz",
    gunzip = True,
    sha256 = "c8941a8094ec3b110a3329ceb54fbe00555f7186aafc464b193d6a3f58c6469e",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_1.6.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_1.6.json.gz",
    gunzip = True,
    sha256 = "c7ba6c2a20a531f119a53685c0f7f106001de23d9b030e7098068fd61fffb22d",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_2.5.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_2.5.json.gz",
    gunzip = True,
    sha256 = "8ac4ee238b9c1197867651428bb8b27a20065d46e2e67102f01a538cee14ea8b",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_4.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_4.json.gz",
    gunzip = True,
    sha256 = "5e5b1a1d8a7d73fbb01100668b769729268a36ff2a3aea840279aaf8ef22403e",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_6.3.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_6.3.json.gz",
    gunzip = True,
    sha256 = "d1949af50d30d65d187ab83d188c4b3721d12d32ff38bb3f1e9ac6f7ea84bb09",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_10.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_10.json.gz",
    gunzip = True,
    sha256 = "0a3453450c25b3db9d70cf171b8c9bb5cefe4c8ef43f0313968fa00eeaa71f22",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_16.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_16.json.gz",
    gunzip = True,
    sha256 = "196172d9d6dd2106acc9dc186d5f594dce2eae4103fafb0d08c4fa952c0ba12c",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_25.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_25.json.gz",
    gunzip = True,
    sha256 = "05c44e4907b0a2414148c3a8ad506d6862f86cb6903f28ee6e54fd377c628c3c",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_40.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_40.json.gz",
    gunzip = True,
    sha256 = "a17f0d4d7003be28a090cc5a512f513deabac86f2f1270f8750f23aedb9107b6",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_63.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_63.json.gz",
    gunzip = True,
    sha256 = "2042531fc1675d2a9ee3afa9744e23eb95610f7c0a7b4a0cc95b0984fd5a9485",
)

gcs_fetch(
    name = "oktoberkite/hover_model_okt_wind_10_no_wake_model.json",
    gcs_path = "gs://gcp-public-data-makani-deps/deps/aero_databases/oktoberkite/20191211_hover_model_okt01_wind_10_no_wake_model.json.gz",
    gunzip = True,
    sha256 = "2ec01454d040de373a931f883e11146ddad9f6928002dc6cc3f57724e475b1b1",
)
