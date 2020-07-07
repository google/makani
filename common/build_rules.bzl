"""This module contains rules for generating data files using Octave."""

def _octave_data_internal_impl(ctx):
    m_path = ctx.file.m_file.path
    last_slash = m_path.rfind("/")
    m_dir = m_path[:last_slash]
    m_function = m_path[(last_slash + 1):-2]

    cmd = ("PATH=/usr/bin octave -q --path %s --eval " % m_dir +
           "\"rand('seed', 1); randn('seed', 1); \"%s\"('\"%s\"');\"" % (
               m_function,
               ctx.outputs.output.path,
           ))

    ctx.actions.run_shell(
        inputs = [ctx.file.m_file],
        outputs = [ctx.outputs.output],
        command = cmd,
        mnemonic = "Generating",
    )

_octave_data_internal = rule(
    implementation = _octave_data_internal_impl,
    attrs = {
        "m_file": attr.label(allow_single_file = [".m"]),
        "output": attr.output(),
    },
)

def octave_data(m_file, output):
    """Generates a data file using Octave.

    Args:
        m_file: .m file for Octave to run.
        output: Name of the output file.
    """
    _octave_data_internal(
        name = "gen_" + output.replace("/", "_").replace(".", "_"),
        m_file = m_file,
        output = output,
    )
