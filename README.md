# SimulationManager
By: Floris Heinen, October 2021

A c++ manager that abstracts the connection to a simulator that follows the [SSL simulation protocol](https://github.com/RoboCup-SSL/ssl-simulation-protocol). This manager handles all the underlying logic and networking necessary to connect to the simulator, resulting in only 4 functions:
* `sendRobotControlCommand(command, team)`
    
    This will send the given command that specifies robot control to the simulator.
* `sendConfigurationCommand(command)`

    This will send a configuration command to the simulator to configure its settings.
* `setRobotControlFeedbackCallback(function)`
  
    This will set the callback for receiving feedback to a robot control command.
* `setConfigurationFeedbackCallback(function)`

    This will set the callback for receiving feedback to a configuration command.

According to the [SSL simulation protocol](https://github.com/RoboCup-SSL/ssl-simulation-protocol), bidirectional UDP connections should be used. GrSim, however, does not implement this (as of Oct 2021), so the feedback is sent on a different port than on which the robot control messages should be sent.

## Requirements
* Protobuf > 3.9.1
* QtNetworking

## TODOs
* Implement configuration commands that support changing the dimension properties of the field
