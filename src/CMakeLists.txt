add_library(simManager "SimulationManager.cpp")
target_link_libraries(simManager simManagerIncludes simManagerProto Qt5::Network)