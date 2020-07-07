def _xfoil_object_impl(ctx):
  """Implementation of the xfoil_object rule."""
  src = ctx.files.src[0]
  if src.path.endswith(".f"):
    compiler = "/usr/bin/gfortran"
  else:
    compiler = "/usr/bin/gcc"

  command = " ".join(
      [compiler]
      + ["-D" + d for d in ctx.attr.defines]
      + ctx.attr.copts
      + ["-c", ctx.files.src[0].path,
         "-o", ctx.outputs.object.path]
  )

  ctx.action(
      inputs = ctx.files.src + ctx.files.includes,
      outputs = [ctx.outputs.object],
      command = command,
  )


# xfoil_object: Produces a .o file from a source file.
#
# Args:
#   src: Single source file from which to build the object.
#   defines: Macro definitions (will be prefaced with "-D").
#   includes: Files included by the source file.
#   copts: Compilation options.
xfoil_object = rule(
    implementation=_xfoil_object_impl,
    attrs={
        "src": attr.label(allow_files=FileType([".f", ".c"])),
        "defines": attr.string_list(),
        "includes": attr.label_list(allow_files=FileType([".inc", ".INC"])),
        "copts": attr.string_list(),
    },
    outputs = {
        "object": "%{name}.o",
    }
)


def internal_xfoil_archive_impl(ctx):
  """Implementation of the internal_xfoil_archive rule."""
  command = " ".join(["/usr/bin/ar cr", ctx.outputs.archive.path]
                     + [obj.path for obj in ctx.files.objects]
                     + ["&& ranlib", ctx.outputs.archive.path])
  ctx.action(
      inputs = ctx.files.objects,
      outputs = [ctx.outputs.archive],
      command = command,
 )


# Produces a .a file from a set of object files.
#
# Args:
#   objects: List of .o files.
internal_xfoil_archive = rule(
  implementation=internal_xfoil_archive_impl,
  attrs={
      "objects": attr.label_list(allow_files=FileType([".o"])),
  },
  outputs = {
      "archive": "lib%{name}.a",
  },
)


def xfoil_archive(name, srcs, includes=[], defines=[], copts=[]):
  """Produces an archive file from source files.

  Args:
    srcs: Source files to be compiled into the archive.
    includes: Files included by the source files.
    defines: Macro definitions (will be prefaced with "-D").
    copts: Compilation options.
  """

  objects = []
  for src in srcs:
    target_name = "_" + name + "/" + src[:-2]
    xfoil_object(
        name = target_name,
        src = src,
        defines = defines,
        includes = includes,
        copts = copts,
    )
    objects += [target_name + ".o"]

  internal_xfoil_archive(
      name = name,
      objects = objects,
  )


def internal_xfoil_fortran_binary_impl(ctx):
  """Implementation of the internal_xfoil_fortran_binary rule."""
  command = " ".join(
      ["/usr/bin/gfortran -B/usr/bin -fdefault-real-8 -O2"]
      + [o.path for o in ctx.files.objects]
      + ["-o", ctx.outputs.executable.path, "-rdynamic"]
      + [a.path for a in ctx.files.archives]
      + ["/usr/lib/x86_64-linux-gnu/libX11.so"]
  )
  ctx.action(
      inputs = ctx.files.objects + ctx.files.archives,
      outputs = [ctx.outputs.executable],
      command = command,
  )


# xfoil_fortran_binary: Compiles a binary from archives and objects.
#
# Args:
#   archives: List of .a files.
#   objects: List of .o files.
internal_xfoil_fortran_binary = rule(
    implementation=internal_xfoil_fortran_binary_impl,
    attrs={
        "archives": attr.label_list(allow_files=FileType([".a"])),
        "objects": attr.label_list(allow_files=FileType([".o"])),
    },
    executable=True,
)


def xfoil_fortran_binary(name, srcs, archives, includes=[], defines=[], copts=[]):
  """Compiles a binary from Fortran source.

  Args:
    srcs: Source files.
    archives: Archives that the binary depends on. These must be specified
        in proper order for the linker.
    includes: Files included by the source files.
    defines: Macro definitions with which to compile the source files
        (will be prefaced with "-D").
    copts: Compilation options for the source files.
  """

  objects = []
  for src in srcs:
    target_name = "_%s/%s" % (name, src.rpartition('.')[0])
    xfoil_object(
        name = target_name,
        src = src,
        defines = defines,
        includes = includes,
        copts = copts,
    )
    objects += [target_name + ".o"]

  internal_xfoil_fortran_binary(
      name = name,
      objects = objects,
      archives = archives,
  )
