"""This module contains the configuration for the CC toolchain."""

load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")
load(
    "@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl",
    "action_config",
    "feature",
    "flag_group",
    "flag_set",
    "tool",
    "tool_path",
    "with_feature_set",
)

def _impl(ctx):
    if (ctx.attr.cpu == "k8"):
        host_system_name = "local"
        target_system_name = "local"
        target_cpu = "k8"
        target_libc = "local"
        abi_version = "local"
        abi_libc_version = "local"
        cc_target_os = "linux"
        if (ctx.attr.compiler == "gcc"):
            toolchain_identifier = "k8_gcc"
            compiler = "gcc"
        elif (ctx.attr.compiler == "clang"):
            toolchain_identifier = "k8_clang"
            compiler = "clang"
        else:
            fail("Unreachable")
    elif (ctx.attr.cpu == "q7"):
        toolchain_identifier = "arm_q7"
        host_system_name = "q7_host_system_name"
        target_system_name = "q7_system_name"
        target_cpu = "q7"
        target_libc = "q7_libc"
        compiler = "q7 compiler"
        abi_version = "q7_abi_version"
        abi_libc_version = "q7_abi_libc_version"
        cc_target_os = "linux"
    elif (ctx.attr.cpu == "tms570"):
        toolchain_identifier = "arm_tms570"
        host_system_name = "tms570_host_system_name"
        target_system_name = "tms570_system_name"
        target_cpu = "tms570"
        target_libc = "tms570_libc"
        compiler = "tms570_compiler"
        abi_version = "tms570_abi_version"
        abi_libc_version = "tms570_abi_libc_version"
        cc_target_os = None
    else:
        fail("Unreachable")

    cc_target_os = None

    builtin_sysroot = None

    all_compile_actions = [
        ACTION_NAMES.c_compile,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.assemble,
        ACTION_NAMES.preprocess_assemble,
        ACTION_NAMES.cpp_header_parsing,
        ACTION_NAMES.cpp_module_compile,
        ACTION_NAMES.cpp_module_codegen,
        ACTION_NAMES.clif_match,
        ACTION_NAMES.lto_backend,
    ]

    all_cpp_compile_actions = [
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.cpp_header_parsing,
        ACTION_NAMES.cpp_module_compile,
        ACTION_NAMES.cpp_module_codegen,
        ACTION_NAMES.clif_match,
    ]

    preprocessor_compile_actions = [
        ACTION_NAMES.c_compile,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.preprocess_assemble,
        ACTION_NAMES.cpp_header_parsing,
        ACTION_NAMES.cpp_module_compile,
        ACTION_NAMES.clif_match,
    ]

    codegen_compile_actions = [
        ACTION_NAMES.c_compile,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.assemble,
        ACTION_NAMES.preprocess_assemble,
        ACTION_NAMES.cpp_module_codegen,
        ACTION_NAMES.lto_backend,
    ]

    all_link_actions = [
        ACTION_NAMES.cpp_link_executable,
        ACTION_NAMES.cpp_link_dynamic_library,
        ACTION_NAMES.cpp_link_nodeps_dynamic_library,
    ]

    if (ctx.attr.cpu == "k8"):
        objcopy_embed_data_action = action_config(
            action_name = "objcopy_embed_data",
            enabled = True,
            tools = [tool(path = "/usr/bin/objcopy")],
        )
    elif (ctx.attr.cpu == "tms570"):
        objcopy_embed_data_action = action_config(
            action_name = "objcopy_embed_data",
            enabled = True,
            tools = [tool(path = "bogus")],
        )
    elif (ctx.attr.cpu == "q7"):
        objcopy_embed_data_action = action_config(
            action_name = "objcopy_embed_data",
            enabled = True,
            tools = [tool(path = "q7/objcopy")],
        )
    else:
        objcopy_embed_data_action = None

    action_configs = [objcopy_embed_data_action]

    if (ctx.attr.cpu == "q7"):
        default_link_flags_feature = feature(
            name = "default_link_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-march=armv7-a",
                                "-mfloat-abi=softfp",
                                "-mfpu=vfpv3-d16",
                                "-mtune=cortex-a9",
                                "-lm",
                                "-static",
                                # Performs cleanup of missing functions.
                                "-Wl,--gc-sections",
                            ],
                        ),
                    ],
                ),
            ],
        )
    elif (ctx.attr.cpu == "tms570"):
        # -nostartfiles:               No system startup files.
        # -static:                     Static linking.
        # -Wl,--gc-sections:           Link-time garbage collection.
        # -Wl,-Map=$@.map:             Link map generation.
        # -Wl,--no-wchar-size-warning: Suppress wchar warnings w.r.t. f021
        #                              library.
        # -ffunction-sections,
        # -fdata-sections:             Individual sections for functions and
        #                              data, useful for link-time garbage
        #                              collection and function execution from
        #                              RAM.
        default_link_flags_feature = feature(
            name = "default_link_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-march=armv7-r",
                                "-mcpu=cortex-r4f",
                                "-mfloat-abi=hard",
                                "-mfpu=vfpv3-d16",
                                "-mbig-endian",
                                "-mthumb",
                                "-nostartfiles",
                                "-static",
                                "-Wl,--gc-sections",
                                "-Wl,--no-wchar-size-warning",
                                "-lm",
                                "-ffunction-sections",
                                "-fdata-sections",
                            ],
                        ),
                    ],
                ),
            ],
        )
    elif (ctx.attr.cpu == "k8"):
        default_link_flags_feature = feature(
            name = "default_link_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [flag_group(flags = [
                        "-lm",
                        "-B/usr/bin/",
                        "-lstdc++",
                    ])],
                ),
            ],
        )
    else:
        default_link_flags_feature = None

    supports_pic_feature = feature(name = "supports_pic", enabled = True)

    objcopy_embed_flags_feature = feature(
        name = "objcopy_embed_flags",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = ["objcopy_embed_data"],
                flag_groups = [flag_group(flags = ["-I", "binary"])],
            ),
        ],
    )

    fastbuild_feature = feature(name = "fastbuild")

    opt_feature = feature(name = "opt")

    dbg_feature = feature(name = "dbg")

    compile_flags = []
    cxx_compile_flags = ["-std=c++11"]

    # We default to a strict set of compiler warnings.  Individual
    # warnings may be turned off for sub-components if necessary.
    #
    # Description of warning flags:
    #
    # -Wbad-function-cast:    Function cast to non-matching type.
    # -Wc++-compat:           Warn about constructs outside of common C and C++.
    # -Wcast-align:           Warn of pointer-casts requiring increase in
    #                             alignment.
    # -Wcast-qual:            Warn if pointer cast removes qualifier (like
    #                             const).
    # -Wconversion:           Warn if implicit type conversion can change value.
    # -Wdisabled-optimization: Warn if requested optimization is disabled.
    # -Wfloat-equal:          Warn if floats are used in equality comparisons.
    # -Wformat=2:             Strictest printf/scanf format checking.
    # -Winline:               Warn if a declared inline function cannot be
    #                             inlined.
    # -Winvalid-pch:          Warn if precompiled header is found an can't be
    #                         used.
    # -Wjump-misses-init:     Warn if switch misses variable
    #                             initialization. (gcc)
    # -Wlogical-op:           Warn of suspicious uses of logical
    #                             operators. (gcc)
    # -Wmissing-include-dirs: Warn if include directory does not exist.
    # -Wmissing-format-attribute: Warn if function pointer should be const/pure.
    # -Wmissing-prototypes:   Warn about missing prototypes.
    # -Wnested-externs:       Warn if extern is declared within a function.
    # -Wredundant-decls:      Warn if something is declared twice in same scope.
    # -Wshadow:               Warn when local variable shadows another variable.
    # -Wstack-protector:      Warn of functions not protected by
    #                             -fstack-protector.
    # -Wstrict-prototypes:    Warn if prototype doesn't declare types.
    # -Wswitch-default:       Warn when a switch does not have a default case.
    # -Wswitch-enum:          Warn when a switch does not include all enums.
    # -Wundef:                Warn if an undefined variable is used in an   #if.
    # -Wunknown-pragmas:      Warn if an unknown   #pragma is encountered.
    # -Wunsuffixed-float-constants: Warn about floats without "f" suffix.
    # -Wwrite-strings:        Warn about copying string constants into char *.
    #
    # TODO: Here are C warning flags that we should add
    # eventually, but we need to resolve issues in the code first:
    #   -Wfloat-equal
    #   -Wbad-function-cast
    #   -Wc++-compat
    #   -Winline
    #   -Wunsuffixed-float-constants
    #
    # TODO: Here are C++ warning flags that we should add
    # eventually, but we need to resolve issues in the code first:
    #   -Wold-style-cast
    #
    # TODO: Here are flags that might be nice to add once we
    # know more about the limits of the hardware:
    #   -Wlarger-than=len
    #   -Wframe-larger-than=len
    #   -Wstack-usage=len
    compile_flags += [
        "-Wall",
        "-Wextra",
        "-Wcast-align",
        "-Wcast-qual",
        "-Wdisabled-optimization",
        "-Winvalid-pch",
        "-Wmissing-format-attribute",
        "-Wredundant-decls",
        "-Wshadow",
        "-Wstack-protector",
        "-Wswitch-default",
        "-Wundef",
        "-Wunknown-pragmas",
        "-Wwrite-strings",
    ]
    if (ctx.attr.cpu != "tms570"):
        # TODO: The following warnings are not yet compatible
        # with the TMS570 code base, but may be added in the future.
        compile_flags += [
            "-pedantic-errors",
            "-Wconversion",
            "-Wformat=2",
            "-Wswitch-enum",
        ]

    cxx_compile_flags += [
        "-Weffc++",
        "-Wfloat-equal",
        "-Winline",
        "-Woverloaded-virtual",
        "-Wsign-promo",
    ]
    if (ctx.attr.compiler == "gcc"):
        # Without -MMD, Bazel will complain that system-style includes using
        # -isystem are undeclared.
        compile_flags += ["-MMD"]

        # GCC-only warnings.
        if (ctx.attr.cpu != "tms570"):
            compile_flags += ["-Wlogical-op"]
        cxx_compile_flags += ["-Wstrict-null-sentinel"]
    elif (ctx.attr.compiler == "clang"):
        # Clang is very aggressive with the sign conversion warning in C++
        # by requiring all array indices to by unsigned, so we disable this
        # for now.
        cxx_compile_flags += ["-Wno-sign-conversion"]

        # TODO(b/140817323): These warnings were generated in the rapid switch
        # to clang 7.  Fix/enable these warnings in the future.
        cxx_compile_flags += ["-Wno-unused-lambda-capture"]
        compile_flags += ["-Wno-conversion"]

    if (ctx.attr.cpu == "k8"):
        default_compile_flags_feature = feature(
            name = "default_compile_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = compile_flags,
                        ),
                    ],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [flag_group(flags = ["-O2"])],
                    with_features = [
                        with_feature_set(features = ["fastbuild"]),
                    ],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-O2",
                                "-g0",
                                "-DNDEBUG",
                                "-Wno-unused-parameter",
                                "-Wno-unused-variable",
                            ],
                        ),
                    ],
                    with_features = [with_feature_set(features = ["opt"])],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [flag_group(flags = ["-O0", "-g"])],
                    with_features = [with_feature_set(features = ["dbg"])],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = cxx_compile_flags,
                        ),
                    ],
                ),
            ],
        )
    elif (ctx.attr.cpu == "tms570"):
        default_compile_flags_feature = feature(
            name = "default_compile_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = compile_flags + [
                                "-march=armv7-r",
                                "-mcpu=cortex-r4f",
                                "-mfloat-abi=hard",
                                "-mfpu=vfpv3-d16",
                                "-mbig-endian",
                                "-mthumb",
                                # Individual sections for functions and data,
                                # useful for link-time garbage collection and
                                # function execution from RAM.
                                "-ffunction-sections",
                                "-fdata-sections",
                            ],
                        ),
                    ],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [flag_group(flags = ["-O2", "-g"])],
                    with_features = [with_feature_set(features = ["dbg"])],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [flag_group(flags = ["-O2"])],
                    with_features = [
                        with_feature_set(features = ["fastbuild"]),
                    ],
                ),
            ],
        )
    elif (ctx.attr.cpu == "q7"):
        default_compile_flags_feature = feature(
            name = "default_compile_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = compile_flags + [
                                "-nostdinc",
                                "-Ithird_party/gsl",
                                "-isystem",
                                "third_party_toolchains/arm_linux_gnueabi/usr/lib/gcc-cross/arm-linux-gnueabi/4.7/include",
                                "-isystem",
                                "third_party_toolchains/arm_linux_gnueabi/usr/lib/gcc-cross/arm-linux-gnueabi/4.7/include-fixed",
                                "-isystem",
                                "third_party_toolchains/arm_linux_gnueabi/usr/arm-linux-gnueabi/include",
                                "-isystem",
                                "third_party_toolchains/arm_linux_gnueabi/usr/include",
                                "-march=armv7-a",
                                "-mfloat-abi=softfp",
                                "-mfpu=vfpv3-d16",
                                "-mtune=cortex-a9",
                                "-no-canonical-prefixes",
                                # These options are required for GSL compilation, allowing us to have
                                # undefined functions. The linker option then performs cleanup.
                                "-ffunction-sections",
                                "-fdata-sections",
                            ],
                        ),
                    ],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [flag_group(flags = ["-O2"])],
                    with_features = [with_feature_set(features = ["fastbuild"])],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = cxx_compile_flags + [
                                "-isystem",
                                "third_party_toolchains/arm_linux_gnueabi/usr/arm-linux-gnueabi/include/c++/4.7.3",
                                "-isystem",
                                "third_party_toolchains/arm_linux_gnueabi/usr/arm-linux-gnueabi/include/c++/4.7.3/arm-linux-gnueabi/sf",
                                "-isystem",
                                "third_party_toolchains/arm_linux_gnueabi/usr/arm-linux-gnueabi/include/c++/4.7.3/backward",
                            ],
                        ),
                    ],
                ),
            ],
        )
    else:
        default_compile_flags_feature = None

    if (ctx.attr.cpu == "tms570"):
        features = [
            default_compile_flags_feature,
            default_link_flags_feature,
            objcopy_embed_flags_feature,
            dbg_feature,
            fastbuild_feature,
        ]
    elif (ctx.attr.cpu == "q7"):
        features = [
            default_compile_flags_feature,
            default_link_flags_feature,
            objcopy_embed_flags_feature,
            fastbuild_feature,
        ]
    elif (ctx.attr.cpu == "k8"):
        features = [
            default_compile_flags_feature,
            default_link_flags_feature,
            supports_pic_feature,
            objcopy_embed_flags_feature,
            fastbuild_feature,
            opt_feature,
            dbg_feature,
        ]
    else:
        fail("Unreachable")

    if (ctx.attr.cpu == "tms570"):
        cxx_builtin_include_directories = []
    elif (ctx.attr.cpu == "q7"):
        cxx_builtin_include_directories = [
            "%package(//third_party_toolchains/)%/arm_linux_gnueabi/usr/arm-linux-gnueabi/include/c++/4.7.3",
            "%package(//third_party_toolchains/)%/arm_linux_gnueabi/usr/arm-linux-gnueabi/include/c++/4.7.3/arm-linux-gnueabi/sf",
            "%package(//third_party_toolchains/)%/arm_linux_gnueabi/usr/arm-linux-gnueabi/include/c++/4.7.3/backward",
        ]
    elif (ctx.attr.cpu == "k8"):
        cxx_builtin_include_directories = [
            "/usr/lib/gcc/",
            "/usr/local/include",
            "/usr/include",
            # For non-sandboxed builds, system headers in /usr/include are
            # visible via the @usr_include repo, and references to them will
            # resolve there. Bazel will complain about undeclared inclusions
            # without this line.
            "%package(@usr_include//)%",
        ]
        if (ctx.attr.compiler == "clang"):
            cxx_builtin_include_directories += [
                "/usr/lib/llvm-7/lib/clang/7.0.1/",
            ]

            # Clang doesn't support -MMD, so -isystem directories must be listed
            # here to prevent Bazel from complaining about undeclared
            # inclusions. And for plain C, Bazel doesn't actually add -isystem
            # options for these, so we have to repeat them in
            # lib/bazel/c_rules.bzl. Which is kind of silly.
            # TODO: The second part of the above is probably fixable.
            cxx_builtin_include_directories += [
                "/usr/include/glib-2.0",
                "/usr/lib/x86_64-linux-gnu/glib-2.0/include",
                "/usr/lib/x86_64-linux-gnu/gtk-2.0/include/gdkconfig.h",
            ]
    else:
        fail("Unreachable")

    artifact_name_patterns = []

    make_variables = []

    if (ctx.attr.cpu == "k8"):
        tool_paths = [
            tool_path(name = "ar", path = "/usr/bin/ar"),
            tool_path(name = "compat-ld", path = "/usr/bin/ld"),
            tool_path(name = "cpp", path = "/usr/bin/cpp"),
            tool_path(name = "dwp", path = "/usr/bin/dwp"),
            tool_path(name = "gcc", path = "/usr/bin/clang-7"),
            tool_path(name = "gcov", path = "/usr/bin/gcov"),
            tool_path(name = "ld", path = "/usr/bin/ld"),
            tool_path(name = "nm", path = "/usr/bin/nm"),
            tool_path(name = "objcopy", path = "/usr/bin/objcopy"),
            tool_path(name = "objdump", path = "/usr/bin/objdump"),
            tool_path(name = "strip", path = "/usr/bin/strip"),
        ]
        if (ctx.attr.compiler == "gcc"):
            tool_paths += [
                tool_path(name = "gcc", path = "/usr/bin/gcc"),
            ]
        elif (ctx.attr.compiler == "clang"):
            tool_paths += [
                tool_path(name = "gcc", path = "/usr/bin/clang-7"),
            ]
    elif (ctx.attr.cpu == "q7"):
        tool_paths = [
            tool_path(name = "ar", path = "q7/ar"),
            tool_path(name = "compat-ld", path = "q7/ld"),
            tool_path(name = "cpp", path = "q7/cpp"),
            tool_path(name = "dwp", path = "q7/dwp"),
            tool_path(name = "gcc", path = "q7/gcc"),
            tool_path(name = "gcov", path = "q7/gcov"),
            tool_path(name = "ld", path = "q7/ld"),
            tool_path(name = "nm", path = "q7/nm"),
            tool_path(name = "objcopy", path = "q7/objcopy"),
            tool_path(name = "objdump", path = "q7/objdump"),
            tool_path(name = "strip", path = "q7/strip"),
        ]
    elif (ctx.attr.cpu == "tms570"):
        tool_paths = [
            tool_path(name = "ar", path = "tms570/ar"),
            tool_path(name = "gcc", path = "tms570/gcc"),
            tool_path(name = "compat-ld", path = "bogus"),
            tool_path(name = "cpp", path = "bogus"),
            tool_path(name = "dwp", path = "bogus"),
            tool_path(name = "gcov", path = "bogus"),
            tool_path(name = "ld", path = "bogus"),
            tool_path(name = "nm", path = "bogus"),
            tool_path(name = "objcopy", path = "bogus"),
            tool_path(name = "objdump", path = "bogus"),
            tool_path(name = "strip", path = "bogus"),
        ]
    else:
        fail("Unreachable")

    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, "Fake executable")
    return [
        cc_common.create_cc_toolchain_config_info(
            ctx = ctx,
            features = features,
            action_configs = action_configs,
            artifact_name_patterns = artifact_name_patterns,
            cxx_builtin_include_directories = cxx_builtin_include_directories,
            toolchain_identifier = toolchain_identifier,
            host_system_name = host_system_name,
            target_system_name = target_system_name,
            target_cpu = target_cpu,
            target_libc = target_libc,
            compiler = compiler,
            abi_version = abi_version,
            abi_libc_version = abi_libc_version,
            tool_paths = tool_paths,
            make_variables = make_variables,
            builtin_sysroot = builtin_sysroot,
            cc_target_os = cc_target_os,
        ),
        DefaultInfo(
            executable = out,
        ),
    ]

cc_toolchain_config = rule(
    implementation = _impl,
    attrs = {
        "compiler": attr.string(default = "gcc", values = ["gcc", "clang"]),
        "cpu": attr.string(mandatory = True, values = ["k8", "q7", "tms570"]),
    },
    provides = [CcToolchainConfigInfo],
    executable = True,
)
