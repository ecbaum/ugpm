# FIND BOOST
find_package(Boost COMPONENTS program_options REQUIRED)


# UGPM demonstration
add_executable(frame_integration src/frame_integration.cpp)

set_target_properties(frame_integration PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES)
target_include_directories( frame_integration PUBLIC
    ${PROJECT_SOURCE_DIR}/external_library/
    ${PROJECT_SOURCE_DIR}/library/
    ${Boost_INCLUDE_DIRS}
    )

target_link_libraries( frame_integration PUBLIC
    celib
    ${Boost_LIBRARIES}
    )




# UGPM paper results
add_executable(paper_metrics src/paper_metrics.cpp)

set_target_properties(paper_metrics PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES)
target_include_directories(paper_metrics PUBLIC
    ${PROJECT_SOURCE_DIR}/external_library/
    ${PROJECT_SOURCE_DIR}/library/
    ${Boost_INCLUDE_DIRS}
    )

target_link_libraries( paper_metrics PUBLIC
    celib
    ${Boost_LIBRARIES}
    )

