add_library(KalmanLib INTERFACE)

target_include_directories(KalmanLib
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>
)

target_compile_features(KalmanLib INTERFACE cxx_std_20)
