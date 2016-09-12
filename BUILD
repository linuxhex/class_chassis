# Bazel (http://bazel.io/) BUILD file for wc_chassis

COPTS = [
    "-std=c++11",
]
LINK_OPTS = []

# wc_chassis
cc_binary(
    name = "wc_chassis",
    srcs = glob([
        "sdk/src/Comm/Comm.cpp",
        "sdk/src/Comm/TimerDiff.cpp",
        "sdk/src/TransferDevice/SPort.cpp",
        "wc_chassis/src/buffer.cpp",
        "wc_chassis/src/protocol.cpp",
        "wc_chassis/src/wc_chassis_mcu.cpp",
        "wc_chassis/src/main.cpp",
        "wc_chassis/src/action.cpp",
        "wc_chassis/src/advertise_service.cpp",
        "wc_chassis/src/init.cpp",
        "wc_chassis/src/parameter.cpp",
        "wc_chassis/src/publish.cpp",
        "wc_chassis/src/subscribe.cpp",
        "wc_chassis/src/common_function.cpp",
        "wc_chassis/src/schedule.cpp",
        "wc_chassis/src/data_process.cpp",
        "sdk/include/*.h",
        "sdk/include/*.hpp",
        "wc_chassis/include/*.h",
    ]),
    includes = [
        "sdk/include",
        "wc_chassis/include",
    ],
    copts = COPTS,
    linkopts = LINK_OPTS,
    defines = [
        "VERIFY_REMTOE_ID=0",
      #  "TEST",
    ],
    linkstatic = True,
    deps = [
        "//gslib:gslib",
        "@io_bazel_rules_ros//ros:roscpp",
        "@io_bazel_rules_ros//ros:tf",
        "@io_bazel_rules_ros//ros:boost",
        "@io_bazel_rules_ros//ros:nav_msgs",
        "//autoscrubber_services:autoscrubber_services",
        "//gs:gs",
    ],
    visibility = ["//visibility:public"],
)
