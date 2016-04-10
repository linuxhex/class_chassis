cc_library(
    name = "tinyxml",
    srcs = [
        "tinyxml.cpp",
        "tinyxmlerror.cpp",
	"tinyxmlparser.cpp",
	"tinystr.cpp",
    ],
    hdrs = glob([
        "tinyxml.h",
        "tinystr.h",
    ]),
    includes = [
        ".",
    ],
    defines = [
        "TIXML_USE_STL",
    ],
    visibility = ["//visibility:public"],
)
