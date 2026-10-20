file(GLOB_RECURSE CPP_SOURCES CONFIGURE_DEPENDS
    "Source/*.cpp"
)

add_library(FlightControllerLib ${CPP_SOURCES})

target_include_directories(FlightControllerLib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>
)

target_compile_features(FlightControllerLib PUBLIC cxx_std_20)
target_link_libraries(FlightControllerLib PUBLIC PubSubLib KalmanLib)
