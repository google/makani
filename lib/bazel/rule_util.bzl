"""This module contains utility functions used for creating rules."""

def gather_headers_and_include_dirs(*args):
    """Gathers header files and include directories from targets.

    Args:
      *args: Each element is either a single .h file or a cc_library target.

    Returns:
      (list of header files, list of include directories)
    """

    targets = []
    for arg in args:
        if type(arg) == "list":
            targets += arg
        else:
            targets += [arg]

    header_files = []
    for target in targets:
        if CcInfo in target:
            header_files += target[CcInfo].compilation_context.headers.to_list()
        elif len(target.files.to_list()) == 1:
            header_files += target.files.to_list()
        else:
            fail("Target %s must be a single .h file or a cc_library target." %
                 target.label)

    include_dirs = []
    for f in header_files:
        if f.root.path:
            include_dir = f.root.path
        else:
            include_dir = "."
        if include_dir not in include_dirs:
            include_dirs += [include_dir]

    return header_files, include_dirs
