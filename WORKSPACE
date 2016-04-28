# ros_comm
git_repository(
    name = "ros_common_git",
    remote = "http://139.196.192.21/ros_common/ros_common.git",
    init_submodules = True,
    commit = "81955d71c70920b04f38702b7998a565c4e3b5ad",
)

bind(
    name = "tf",
    actual = "@ros_common_git//ros/geometry:tf"
)

bind(
    name = "angles",
    actual = "@ros_common_git//ros/angles:angles"
)

bind(
    name = "nav_msgs",
    actual = "@ros_common_git//ros/common_msgs:nav_msgs"
)

bind(
    name = "geometry_msgs",
    actual = "@ros_common_git//ros/common_msgs:geometry_msgs"
)

bind(
    name = "sensor_msgs",
    actual = "@ros_common_git//ros/common_msgs:sensor_msgs"
)

bind(
    name = "visualization_msgs",
    actual = "@ros_common_git//ros/common_msgs:visualization_msgs"
)

bind(
    name = "diagnostic_msgs",
    actual = "@ros_common_git//ros/common_msgs:diagnostic_msgs"
)

bind(
    name = "map_msgs",
    actual = "@ros_common_git//ros/navigation_msgs:map_msgs"
)

bind(
    name = "move_base_msgs",
    actual = "@ros_common_git//ros/navigation_msgs:move_base_msgs"
)

# from ros_comm
git_repository(
    name = "ros_base_git",
    remote = "http://139.196.192.21/ros_common/ros_base.git",
    init_submodules = True,
    commit = "dea81814878bf2754b2be930fdaddfaed3bc4a5d",
)

bind(
    name = "roscpp",
    actual = "@ros_base_git//ros/ros_comm:roscpp"
)

bind(
    name = "message_filters",
    actual = "@ros_base_git//ros/ros_comm:message_filters"
)

bind(
    name = "console_bridge",
    actual = "@ros_base_git//ros/console_bridge:console_bridge"
)

bind(
    name = "rostime",
    actual = "@ros_base_git//ros/roscpp_core:rostime"
)

bind(
    name = "std_msgs",
    actual = "@ros_base_git//ros/std_msgs:std_msgs"
)

# from ros_base
new_http_archive(
  name = "gmock_archive",
  # url = "https://googlemock.googlecode.com/files/gmock-1.7.0.zip",
  url = "http://z.wisagv.com/gmock-1.7.0.zip",
  sha256 = "26fcbb5925b74ad5fc8c26b0495dfc96353f4d553492eb97e85a8a6d2f43095b",
  build_file = "gmock.BUILD",
)

bind(
  name = "gtest",
  actual = "@gmock_archive//:gtest",
)

bind(
  name = "gtest_main",
  actual = "@gmock_archive//:gtest_main",
)

git_repository(
    name = "boost_git",
    remote = "http://139.196.192.21/common/boost_lib.git",
    commit = "0735ab0744cdc258d45af55328fe8c489f134aa2",
)

bind(
    name = "boost",
    actual = "@boost_git//:all"
)

git_repository(
    name = "glog_git",
    remote = "http://139.196.192.21/common/glog.git",
    init_submodules = True,
    commit = "625005d09e687430b4935716a9199ff53be6d2fc",
)

bind(
    name = "glog",
    actual = "@glog_git//:glog"
)

new_http_archive(
  name = "empy_archive",
  url = "http://120.52.72.53/www.alcyone.com/c3pr90ntcsf0/software/empy/empy-latest.tar.gz",
  sha256 = "99f016af2770c48ab57a65df7aae251360dc69a1514c15851458a71d4ddfea9c",
  strip_prefix = "empy-3.3.2",
  build_file = "empy.BUILD",
)

bind(
    name = "empy",
    actual = "@empy_archive//:empy",
)

new_http_archive(
  name = "tinyxml_archive",
  url = "http://nchc.dl.sourceforge.net/project/tinyxml/tinyxml/2.6.2/tinyxml_2_6_2.zip",
  sha256 = "ac6bb9501c6f50cc922d22f26b02fab168db47521be5e845b83d3451a3e1d512",
  strip_prefix = "tinyxml",
  build_file = "tinyxml.BUILD",
)

bind(
    name = "tinyxml",
    actual = "@tinyxml_archive//:tinyxml",
)

git_repository(
    name = "python_git",
    remote = "http://139.196.192.21/common/python_lib.git",
    commit = "084091a77290bd4686ebd0a0a439b9362ca655bd",
)

bind(
    name = "python",
    actual = "@python_git//:all"
)

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
