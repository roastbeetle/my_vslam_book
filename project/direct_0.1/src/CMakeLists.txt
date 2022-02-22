add_library( myslam SHARED
    map.cpp
    mappoint.cpp
    frame.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)