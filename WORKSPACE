workspace(name = "makani")

load(
    "@bazel_tools//tools/build_defs/repo:git.bzl",
    "git_repository",
    "new_git_repository",
)
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

register_toolchains("//lib/bazel:py_toolchain")

new_git_repository(
    name = "googletest",
    build_file = "@//lib/bazel:googletest.BUILD",
    commit = "703bd9caab50b139428cea1aaff9974ebee5742e",
    remote = "https://github.com/google/googletest",
    shallow_since = "1570114335 -0400",
)

new_local_repository(
    name = "usr_include",
    build_file = "lib/bazel/usr_include.BUILD",
    path = "/usr/include",
)

new_local_repository(
    name = "usr_lib",
    build_file = "lib/bazel/usr_lib.BUILD",
    path = "/usr/lib",
)

git_repository(
    name = "bazel_compilation_database",
    commit = "4af36e145ef318e54bf39567f9100608183969a0",
    remote = "https://github.com/grailbio/bazel-compilation-database",
    shallow_since = "1568675563 +0000",
)

# bazel_gazelle tries to install com_github_bazelbuild_buildtools if it's not
# already installed, therefore we load com_github_bazelbuild_buildtools first
# to make sure we use the specific version below.
http_archive(
    name = "com_github_bazelbuild_buildtools",
    sha256 = "d8440da64ac15eca922ca0e8c6772bbb04eaaf3d2f4de387e5bfdb87cecbe9d2",
    strip_prefix = "buildtools-0.28.0",
    url = "https://github.com/bazelbuild/buildtools/archive/0.28.0.zip",
)

http_archive(
    name = "io_bazel_rules_go",
    sha256 = "313f2c7a23fecc33023563f082f381a32b9b7254f727a7dd2d6380ccc6dfe09b",
    urls = [
        "https://storage.googleapis.com/bazel-mirror/github.com/bazelbuild/rules_go/releases/download/0.19.3/rules_go-0.19.3.tar.gz",
        "https://github.com/bazelbuild/rules_go/releases/download/0.19.3/rules_go-0.19.3.tar.gz",
    ],
)

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains", "go_rules_dependencies")

go_rules_dependencies()

go_register_toolchains()

http_archive(
    name = "bazel_gazelle",
    sha256 = "7fc87f4170011201b1690326e8c16c5d802836e3a0d617d8f75c3af2b23180c4",
    urls = [
        "https://storage.googleapis.com/bazel-mirror/github.com/bazelbuild/bazel-gazelle/releases/download/0.18.2/bazel-gazelle-0.18.2.tar.gz",
        "https://github.com/bazelbuild/bazel-gazelle/releases/download/0.18.2/bazel-gazelle-0.18.2.tar.gz",
    ],
)

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")

gazelle_dependencies()

http_archive(
    name = "com_google_protobuf",
    sha256 = "c90d9e13564c0af85fd2912545ee47b57deded6e5a97de80395b6d2d9be64854",
    strip_prefix = "protobuf-3.9.1",
    urls = ["https://github.com/protocolbuffers/protobuf/archive/v3.9.1.zip"],
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

git_repository(
    name = "subpar",
    commit = "35bb9f0092f71ea56b742a520602da9b3638a24f",
    remote = "https://github.com/google/subpar",
    shallow_since = "1557863961 -0400",
)

git_repository(
    name = "rules_python",
    commit = "dd7f9c5f01bafbfea08c44092b6b0c8fc8fcb77f",
    remote = "https://github.com/bazelbuild/rules_python",
    shallow_since = "1582315370 -0500",
)

# This call should always be present.
load("@rules_python//python:repositories.bzl", "py_repositories")

py_repositories()

# This one is only needed if you're using the packaging rules.
load("@rules_python//python:pip.bzl", "pip_import", "pip_repositories")

pip_repositories()

pip_import(
    name = "python_pip",
    extra_pip_args = [
        "-f",
        "/opt/makani/pip",
    ],
    requirements = "//lib/bazel:requirements.txt",
)

load("@python_pip//:requirements.bzl", "pip_install")

pip_install()

http_archive(
    name = "io_bazel_rules_appengine",
    sha256 = "3e21a23c6b70d3bdb43dbc4930349efbe1aadb2184192d9175fb058f787176ab",
    strip_prefix = "rules_appengine-0.0.9",
    url = "https://github.com/bazelbuild/rules_appengine/archive/0.0.9.zip",
)

load(
    "@io_bazel_rules_appengine//appengine:sdk.bzl",
    "appengine_repositories",
)

appengine_repositories()

load(
    "@io_bazel_rules_appengine//appengine:py_appengine.bzl",
    "py_appengine_repositories",
)

py_appengine_repositories()

http_archive(
    name = "bazel_toolchains",
    sha256 = "fc55f4d9ee8e3d9535395717d3ee892116d9bccaa386c447a1cb100d4509affe",
    strip_prefix = "bazel-toolchains-1.2.3",
    urls = [
        "https://github.com/bazelbuild/bazel-toolchains/archive/1.2.3.tar.gz",
    ],
)
