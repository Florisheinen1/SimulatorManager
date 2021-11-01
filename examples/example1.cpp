// By: Floris Heinen
#include <iostream>
#include <chrono>
#include <thread>
#include "SimulatorManager.hpp"

// TODO: Debug: change property -> change in local speed direction

namespace rtt::robothub::simulation {
    ConfigurationCommand createFormationCommand() {
        ConfigurationCommand command;

        for (int i = 0; i < 11; ++i) {
            float d = (i * 0.5f) + 0.5f;
            command.addRobotLocation(i, true, d, -5.3f, 0.0f, 0.0f, 2.0f, 0.0f, true, false);
            command.addRobotLocation(i, false, -d, -5.3f, 0.0f, 0.0f, 2.0f, 0.0f, true, false);
        }

        return command;
    }

    ConfigurationCommand createWeirdConfigCommand() {
        ConfigurationCommand command;

        //command.addRobotLocation(3, true, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, true, false);
        RobotProperties property {.radius = 0.5f};
        command.addRobotSpecs(3, true, property);
        return command;
    }
}

void onControlFeedback(rtt::robothub::simulation::RobotControlFeedback& feedback) {
    //std::cout << "Received feedback in example!" << std::endl;
    std::cout << "---> Feedback for team: " << (feedback.isTeamYellow ? "yellow" : "blue") << ", errors: " << feedback.simulationErrors.size() << std::endl;
}
void onConfigurationFeedback(rtt::robothub::simulation::ConfigurationFeedback& feedback) {
    std::cout << "---> Configuration feedback with: " << feedback.simulationErrors.size() << " errors!" << std::endl;
}

int main() {
    using namespace rtt::robothub::simulation;

    {
        // GrSim does not allow a bidirectional udp connection, so it uses different ports for the feedback
        SimulatorNetworkConfiguration config = { .blueFeedbackPort = 30011,
                                                 .yellowFeedbackPort = 30012,
                                                 .configurationFeedbackPort = 30013 };
        
        std::unique_ptr<SimulatorManager> manager;

        try {
            manager = std::make_unique<SimulatorManager>(config);
        } catch (FailedToBindPortException e) {
            std::cout << "Error: " << e.what() << std::endl;
            return -1;
        }

        std::cout << "Setting callback... ";
        manager->setRobotControlFeedbackCallback(onControlFeedback);
        manager->setConfigurationFeedbackCallback(onConfigurationFeedback);
        std::cout << "Done!" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Sending robot command to blue... ";
        RobotControlCommand command;
        command.addRobotControlWithLocalSpeeds(9, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        manager->sendRobotControlCommand(command, false);
        std::cout << "Done!" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Sending robot command to yellow... ";
        RobotControlCommand command2;
        command2.addRobotControlWithLocalSpeeds(0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        command2.addRobotControlWithLocalSpeeds(2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        manager->sendRobotControlCommand(command2, true);
        std::cout << "Done!" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Sending configuration command... ";
        ConfigurationCommand command3;
        command3.setSimulationSpeed(10);
        manager->sendConfigurationCommand(command3);
        std::cout << "Done!" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
        
    }
    std::cout << "Closed!" << std::endl;

    return 0;
}