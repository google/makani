"""This module contains rules for generating param files."""

def gen_param_file(out, src, key):
    native.genrule(
        name = out + "_rule",
        srcs = [src],
        tags = ["arch:tms570"],
        outs = [out],
        output_to_bindir = True,
        tools = ["//avionics/firmware/params:param_util"],
        cmd = " ".join([
            "$(location //avionics/firmware/params:param_util)",
            "--input $(location " + src + ")",
            "--output $(location " + out + ")",
            "--yaml_key " + key,
        ]),
    )
