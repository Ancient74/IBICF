add_library(PubSubLib INTERFACE)

target_include_directories(PubSubLib
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>
)

target_compile_features(PubSubLib INTERFACE cxx_std_20)
