// By: Floris Heinen
#pragma once

#include <QtNetwork>
#include <string>

#include <google/protobuf/message.h>

#include "proto/ssl_simulation_control.pb.h"
#include "proto/ssl_simulation_config.pb.h"
#include "proto/ssl_vision_geometry.pb.h"
#include "proto/ssl_simulation_robot_control.pb.h"

namespace rtt::robothub::simulation {
    constexpr int PACKET_SERIALIZATION_BUFFER_SIZE = 1024;
    
    class RobotControlCommand;
    class ConfigurationCommand;

/*  This class can send two types of commands to any simulator using the SSL_simulation_protocol:
    Robot control commands for controlling robots and configuration commands to configure the sim.
    Use mutliple CommandSenders for different ports, for example one for blue team and one for
    yellow team if you want to control both teams.
    Configuration settings can also be sent, but that needs a unique port too. */
    class SimulationManager {
    public:
        SimulationManager(const std::string& simIpAddress, int blueControlPort, int yellowControlPort, int configurationPort);

        // Both send functions return amount of bytes sent, return -1 if error occured.
        int sendRobotControlCommand(RobotControlCommand& robotControlCommand, bool isTeamYellow);
        int sendConfigurationCommand(ConfigurationCommand& configurationCommand);
        
    private:
        std::array<char, PACKET_SERIALIZATION_BUFFER_SIZE> packetSerializationBuffer;
    
        QUdpSocket blueControlSocket;
        QUdpSocket yellowControlSocket;
        QUdpSocket configurationSocket;
        
        QHostAddress simIpAddress;
        int blueControlPort;
        int yellowControlPort;
        int configurationPort;

        // Returns the amount of bytes sent. Returns -1 if error occured.
        int sendPacket(google::protobuf::Message& packet, QUdpSocket& socket, int port);
    };

/*  This class contains the information for moving robots.
    It immediately stores the given information into a packet that can be sent to the simulator.
    Robots can be controlled by either wheel speeds, local velocities or global velocties.
    Local velocities are speeds like forward and sideway speeds, while global velocties are
    speeds relative to the field. */
    class RobotControlCommand {
    public:
        void addRobotControlWithWheelSpeeds(
            int robotId,
            float kickSpeed,
            float kickAngle,
            float dribblerSpeed,
            float frontRightWheelVelocity,
            float backRightWheelVelocity,
            float backLeftWheelVelocity,
            float frontLeftWheelVelocity);
        void addRobotControlWithLocalSpeeds(
            int robotId,
            float kickSpeed,
            float kickAngle,
            float dribblerSpeed,
            float forwardVelocity,
            float leftVelocity,
            float angularVelocity);
        void addRobotControlWithGlobalSpeeds(
            int robotId,
            float kickSpeed,
            float kickAngle,
            float dribblerSpeed,
            float xVelocity,
            float yVelocity,
            float angularVelocity);
        proto::RobotControl& getPacket();
    private:
        proto::RobotControl controlCommand;
    };

    typedef struct RobotProperties {
        // Units in meters, kilograms, degrees, m/s or m/s^2.
        float radius = 0.09f;
        float height = 0.15f;
        float mass = 2.0f;
        float maxKickSpeed = 6.5f;
        float maxChipSpeed = 6.5f;
        float centerToDribblerDistance = 0.09;
        // Robot limits
        float maxAcceleration = 3.0f;
        float maxAngularAcceleration = 3.0f;
        float maxDeceleration = 3.0f;
        float maxAngularDeceleration = 3.0f;
        float maxVelocity = 3.0f;
        float maxAngularVelocity = 3.0f;
        // Wheel angles. Counter-clockwise starting from dribbler
        float frontRightWheelAngle = 300.0f;
        float backRightWheelAngle = 210.0f;
        float backLeftWheelAngle = 150.0f;
        float frontLeftWheelAngle = 60.0f;
    } RobotProperties;

/*  This class contains command information to configure and setup the simulator.
    It immediately stores the given information into a packet that can be sent to the simulator.
    Things to configure are robot positions, velocities, ball positions, physical properties of
    the robot, simulator speed, the vision port.
    TODO: Also add functionality to change the field geometry properties
    TODO: GrSim ignores robotId when setting robot properties, resulting in all robots from the
    same team having the same properties. This is grSim's fault, so go make a pull request to
    grSim that fixes this faulty behavior. */
    class ConfigurationCommand {
    public:

        // VelocityInRolling transforms the velocities into spinning engergy.
        // ByForce will set velocities accordingly to make sure the ball ends
        // up at the specified coordinates.
        void setBallLocation(float x, float y, float z,
                            float xVelocity, float yVelocity, float zVelocity,
                            bool velocityInRolling, bool teleportSafely, bool byForce);
        // Orientation is a global rotation relative to the field.
        // shouldBePresentOnField will make a robot (dis)appear accordingly.
        void addRobotLocation(int id, bool isFromTeamYellow,
                            float x, float y,
                            float xVelocity, float yVelocity, float angularVelocity,
                            float orientation, bool shouldBePresentOnField, bool byForce);
        void setSimulationSpeed(float speed);
        void addRobotSpecs(int id, bool isFromTeamYellow, RobotProperties& robotProperties);
        void setVisionPort(int port);
        
        proto::SimulatorCommand& getPacket();
        
    private:
        proto::SimulatorCommand configurationCommand;
    };
}