file(GLOB_RECURSE CPP_SOURCES CONFIGURE_DEPENDS
    "Source/*.cpp"
)

add_library(CrsfLib ${CPP_SOURCES})

target_include_directories(CrsfLib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>
)

target_compile_features(CrsfLib PUBLIC cxx_std_20)
