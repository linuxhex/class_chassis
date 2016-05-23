# ros_comm
git_repository(
    name = "io_bazel_rules_ros",
    remote = "http://git.gs-robot.com/bazel-build/rules_ros.git",
    commit = "d6dfa4515b836820f0d3ebacfadb8179dd957ab1",
)
load("@io_bazel_rules_ros//ros:ros.bzl", "ros_repositories")
 
ros_repositories()

# TODO(chenkan): use LibYAML bindings
new_http_archive(
  name = "pyyaml_archive",
  url = "http://pyyaml.org/download/pyyaml/PyYAML-3.11.tar.gz",
  sha256 = "c36c938a872e5ff494938b33b14aaa156cb439ec67548fcab3535bb78b0846e8",
  strip_prefix = "PyYAML-3.11",
  build_file = "pyyaml.BUILD",
)

bind(
    name = "pyyaml",
    actual = "@pyyaml_archive//:pyyaml",
)
