# Create a CMAKE interface target that contains this include folder.
add_library(simManagerIncludes INTERFACE)
target_include_directories(simManagerIncludes INTERFACE "${CMAKE_CURRENT_LIST_DIR}")

# The header files here also need access to proto and the QtNetwork thing, so add these libs too
target_link_libraries(simManagerIncludes INTERFACE simManagerProto Qt5::Network)