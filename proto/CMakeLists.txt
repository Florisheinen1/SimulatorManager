# Generate the protobuf files
protobuf_generate_cpp(SIM_PROTO_SRCS SIM_PROTO_HDRS
"${CMAKE_CURRENT_LIST_DIR}/proto/ssl_simulation_robot_control.proto"
"${CMAKE_CURRENT_LIST_DIR}/proto/ssl_simulation_control.proto"
"${CMAKE_CURRENT_LIST_DIR}/proto/ssl_gc_common.proto"
"${CMAKE_CURRENT_LIST_DIR}/proto/ssl_simulation_config.proto"
"${CMAKE_CURRENT_LIST_DIR}/proto/ssl_simulation_error.proto"
"${CMAKE_CURRENT_LIST_DIR}/proto/ssl_vision_geometry.proto")

# Create the target that contains the necessary proto files
add_library(simManagerProto STATIC "${SIM_PROTO_SRCS}" "${SIM_PROTO_HDRS}")
target_link_libraries(simManagerProto PUBLIC "${PROTOBUF_LIBRARIES}")
target_include_directories(simManagerProto PUBLIC "${PROTOBUF_INCLUDE_DIRS}" "${CMAKE_CURRENT_BINARY_DIR}/..")
