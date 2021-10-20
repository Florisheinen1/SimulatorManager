// By: Floris Heinen
#include <iostream>
#include <chrono>
#include <thread>
#include "SimulationManager.hpp"

// TODO: Debug property -> change in local speed direction

namespace rtt::robothub::simulation {
    ConfigurationCommand createConfigurationCommand1() {
        ConfigurationCommand command;
        command.setBallLocation(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, true, true, true);

        RobotProperties rps { .radius = 0.2f };
        //command.addRobotSpecs(3, false, rps);
        return command;
    }
    ConfigurationCommand createFormationCommand() {
        ConfigurationCommand command;

        for (int i = 0; i < 11; ++i) {
            float d = (i * 0.5f) + 0.5f;
            command.addRobotLocation(i, true, d, -5.3f, 0.0f, 0.0f, 2.0f, 0.0f, true, false);
            command.addRobotLocation(i, false, -d, -5.3f, 0.0f, 0.0f, 2.0f, 0.0f, true, false);
        }

        return command;
    }

    ConfigurationCommand createWeirdConfig() {
        ConfigurationCommand command;

        //command.addRobotLocation(3, true, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, true, false);
        RobotProperties property {.radius = 0.5f};
        command.addRobotSpecs(3, true, property);
        return command;
    }

    RobotControlCommand createControlCommand() {
        RobotControlCommand command;
        command.addRobotControlWithLocalSpeeds(9, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f);
        return command;
    }

}
int main() {
    
    //Sim::CommandSender generalCommander("127.0.0.1", 20011);
    rtt::robothub::simulation::SimulationManager simManager("127.0.0.1", 10301, 10302, 10300);
    //Sim::CommandSender yellowCommander("127.0.0.1", 10302);
    //Sim::CommandSender simCommander("127.0.0.1", 10300);
    
    //Sim::ConfigurationCommand weirdCommand = createWeirdConfig();
    //simCommander.sendConfigurationCommand(weirdCommand);

    std::cout << "Sending command... ";
    rtt::robothub::simulation::RobotControlCommand command = rtt::robothub::simulation::createControlCommand();
    rtt::robothub::simulation::ConfigurationCommand config = rtt::robothub::simulation::createConfigurationCommand1();

    simManager.sendConfigurationCommand(config);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    simManager.sendRobotControlCommand(command, false);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    simManager.sendRobotControlCommand(command, true);

    std::cout << "Sent." << std::endl;
    //Sim::CommandSender feedbackSender("127.0.0.1", 30011);
    //feedbackSender.sendRobotControlCommand(command);

    return 0;
}