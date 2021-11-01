add_library(simManager "SimulatorManager.cpp")
target_link_libraries(simManager simManagerIncludes simManagerProto Qt5::Network Qt5::Widgets)