"""This module contains miscellaneous rules."""

def _error_impl(ctx):
    fail(ctx.attr.message)

# Produces an error when the rule is built.
#
# This is used to replace a rule that is no longer supported with an error
# message.
error = rule(
    implementation = _error_impl,
    attrs = {
        "message": attr.string(),
    },
)

def gcs_fetch(name, gcs_path, sha256, executable = False, gunzip = False):
    """Fetches a file from Google Cloud Storage with local caching.

    Args:
      name: Local name for the file.
      gcs_path: Path of the file on GCS.
      sha256: SHA-256 checksum of the file.
      gunzip: Whether to gunzip the file. If true, gcs_path must end with ".gz".
      executable: Whether the file is executable.
    """
    native.genrule(
        name = name + "-genrule",
        outs = [name],
        executable = executable,
        tools = [
            "//lib/bazel:gcs_fetch",
        ],
        cmd = " ".join([
            "$(location //lib/bazel:gcs_fetch)",
            "--gcs_path",
            gcs_path,
            "--package",
            native.package_name(),
            "--target",
            name,
            "--sha256",
            sha256,
            "--gunzip" if gunzip else "--nogunzip",
            "--output_path",
            "$@",
        ]),
        local = 1,
        output_to_bindir = True,
    )
