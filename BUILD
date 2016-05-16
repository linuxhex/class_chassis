# Bazel (http://bazel.io/) BUILD file for wc_chassis

COPTS = [
    "-std=c++11",
]
LINK_OPTS = []

# costmap_2d
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
    ],
    linkstatic = True,
    deps = [
        "//gslib:gslib",
        "//external:roscpp",
        "//external:tf",
        "//external:boost",
        "//external:nav_msgs",
        "//autoscrubber_services:autoscrubber_services",
        "//gs:gs",
    ],
    visibility = ["//visibility:public"],
)
