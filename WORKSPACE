workspace(name = "service_robot_device")

new_http_archive(
    name = "cryptopp_http",
    url = "https://github.com/weidai11/cryptopp/archive/CRYPTOPP_5_6_3.tar.gz",
    sha256 = "ed93f969312d956bdd95db8320ec9573c38f4fe98960a5c583693bddf0eb0654",
    strip_prefix = "cryptopp-CRYPTOPP_5_6_3",
    build_file = "build_file/BUILD.cryptopp",
)

bind(
    name = "cryptopp",
    actual = "@cryptopp_http//:cryptopp",
)

new_http_archive(
    name = "yaml_cpp_http",
    url = "https://github.com/jbeder/yaml-cpp/archive/release-0.5.3.tar.gz",
    sha256 = "ac50a27a201d16dc69a881b80ad39a7be66c4d755eda1f76c3a68781b922af8f",
    strip_prefix = "yaml-cpp-release-0.5.3",
    build_file = "build_file/BUILD.yaml-cpp",
)

bind(
    name = "yaml-cpp",
    actual = "@yaml_cpp_http//:yaml-cpp",
)

git_repository(
    name = "io_bazel_rules_ros",
    remote = "http://git.gs-robot.com/bazel-build/rules_ros.git",
    commit = "d6dfa4515b836820f0d3ebacfadb8179dd957ab1",
)
load("@io_bazel_rules_ros//ros:ros.bzl", "ros_repositories")

ros_repositories()
