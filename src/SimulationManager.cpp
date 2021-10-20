// By: Floris Heinen
#include <iostream>

#include <QtNetwork>
#include <string>

#include <google/protobuf/message.h>

#include "SimulationManager.hpp"

namespace rtt::robothub::simulation {
    SimulationManager::SimulationManager(const std::string& simIpAddress, int blueControlPort, int yellowControlPort, int configurationPort) {
        this->simIpAddress = QHostAddress(QString::fromStdString(simIpAddress));
        
        this->blueControlPort = blueControlPort;
        this->yellowControlPort = yellowControlPort;
        this->configurationPort = configurationPort;

        this->blueControlSocket.bind(QHostAddress::AnyIPv4, blueControlPort, QUdpSocket::ShareAddress);
        this->yellowControlSocket.bind(QHostAddress::AnyIPv4, yellowControlPort, QUdpSocket::ShareAddress);
        this->configurationSocket.bind(QHostAddress::AnyIPv4, configurationPort, QUdpSocket::ShareAddress);
    }

    int SimulationManager::sendRobotControlCommand(RobotControlCommand& robotControlCommand, bool isTeamYellow) {
        int bytesSent;
        if (isTeamYellow)
            bytesSent = this->sendPacket(robotControlCommand.getPacket(), this->yellowControlSocket, this->yellowControlPort);
        else
            bytesSent = this->sendPacket(robotControlCommand.getPacket(), this->blueControlSocket, this->blueControlPort);
        return bytesSent;
    }

    int SimulationManager::sendConfigurationCommand(ConfigurationCommand& configurationCommand) {
        return this->sendPacket(configurationCommand.getPacket(), this->configurationSocket, this->configurationPort);
    }

    int SimulationManager::sendPacket(google::protobuf::Message& packet, QUdpSocket& socket, int port) {
        int packetByteSize = packet.ByteSizeLong();
        int bytesSent;

        // If the packet can fit in the serialization buffer, use the buffer. Else, use slower approach with byteArray
        if (packetByteSize < this->packetSerializationBuffer.size()) {
            // Serialize packet into the buffer
            packet.SerializeToArray(this->packetSerializationBuffer.data(), this->packetSerializationBuffer.size());
            // And send the contents of the buffer to the simulator
            bytesSent = socket.writeDatagram(this->packetSerializationBuffer.data(), packetByteSize, this->simIpAddress, port);
        } else {
            // Create a byteArray where the packet can be serialized into
            QByteArray datagram;
            datagram.resize(packetByteSize);
            packet.SerializeToArray(datagram.data(), datagram.size());
            // Send contents to simulator
            bytesSent = socket.writeDatagram(datagram, this->simIpAddress, port);
        }

        return bytesSent;
    }

    void ConfigurationCommand::setBallLocation(float x, float y, float z,
                        float xVelocity, float yVelocity, float zVelocity,
                        bool velocityInRolling, bool teleportSafely, bool byForce) {
        proto::TeleportBall* tpBallCommand = this->configurationCommand.mutable_control()->mutable_teleport_ball();
        tpBallCommand->set_x(x);
        tpBallCommand->set_y(y);
        tpBallCommand->set_z(z);
        tpBallCommand->set_vx(xVelocity);
        tpBallCommand->set_vy(yVelocity);
        tpBallCommand->set_vz(zVelocity);
        tpBallCommand->set_roll(velocityInRolling);
        tpBallCommand->set_teleport_safely(teleportSafely);
        tpBallCommand->set_by_force(byForce);
    }
    void ConfigurationCommand::addRobotLocation(int id, bool isFromTeamYellow,
                        float x, float y,
                        float xVelocity, float yVelocity, float angularVelocity,
                        float orientation, bool shouldBePresentOnField, bool byForce) {
        proto::TeleportRobot* tpRobotCommand = this->configurationCommand.mutable_control()->add_teleport_robot();
        tpRobotCommand->mutable_id()->set_id(id);
        tpRobotCommand->mutable_id()->set_team(isFromTeamYellow ? proto::Team::YELLOW : proto::Team::BLUE);
        tpRobotCommand->set_x(x);
        tpRobotCommand->set_y(y);
        tpRobotCommand->set_v_x(xVelocity);
        tpRobotCommand->set_v_y(yVelocity);
        tpRobotCommand->set_v_angular(angularVelocity);
        tpRobotCommand->set_orientation(orientation);
        tpRobotCommand->set_present(shouldBePresentOnField);
        tpRobotCommand->set_by_force(byForce);
    }
    void ConfigurationCommand::setSimulationSpeed(float speed) {
        this->configurationCommand.mutable_control()->set_simulation_speed(speed);
    }
    void ConfigurationCommand::addRobotSpecs(int id, bool isFromTeamYellow, RobotProperties& robotProperties) {
        proto::RobotSpecs* specs = this->configurationCommand.mutable_config()->add_robot_specs();

        specs->mutable_id()->set_id(id);
        specs->mutable_id()->set_team(isFromTeamYellow ? proto::Team::YELLOW : proto::Team::BLUE);
        specs->set_radius(robotProperties.radius);
        specs->set_height(robotProperties.height);
        specs->set_mass(robotProperties.mass);
        specs->set_max_linear_kick_speed(robotProperties.maxKickSpeed);
        specs->set_max_chip_kick_speed(robotProperties.maxChipSpeed);
        specs->set_center_to_dribbler(robotProperties.centerToDribblerDistance);
        // Set limits
        specs->mutable_limits()->set_acc_speedup_absolute_max(robotProperties.maxAcceleration);
        specs->mutable_limits()->set_acc_speedup_angular_max(robotProperties.maxAngularAcceleration);
        specs->mutable_limits()->set_acc_brake_absolute_max(robotProperties.maxDeceleration);
        specs->mutable_limits()->set_acc_brake_angular_max(robotProperties.maxAngularDeceleration);
        specs->mutable_limits()->set_vel_absolute_max(robotProperties.maxVelocity);
        specs->mutable_limits()->set_vel_angular_max(robotProperties.maxAngularVelocity);
        // Set wheel angles
        specs->mutable_wheel_angles()->set_front_right(robotProperties.frontRightWheelAngle);
        specs->mutable_wheel_angles()->set_back_right(robotProperties.backRightWheelAngle);
        specs->mutable_wheel_angles()->set_back_left(robotProperties.backLeftWheelAngle);
        specs->mutable_wheel_angles()->set_front_left(robotProperties.frontLeftWheelAngle);
    }
    void ConfigurationCommand::setVisionPort(int port) {
        this->configurationCommand.mutable_config()->set_vision_port(port);
    }
    proto::SimulatorCommand& ConfigurationCommand::getPacket() {
        return this->configurationCommand;
    }

    void RobotControlCommand::addRobotControlWithWheelSpeeds(
                int robotId,
                float kickSpeed,
                float kickAngle,
                float dribblerSpeed,
                float frontRightWheelVelocity,
                float backRightWheelVelocity,
                float backLeftWheelVelocity,
                float frontLeftWheelVelocity) {
        proto::RobotCommand* command = this->controlCommand.add_robot_commands();
        command->set_id(robotId);
        command->set_kick_speed(kickSpeed);
        command->set_kick_angle(kickAngle);
        command->set_dribbler_speed(dribblerSpeed);
        
        proto::MoveWheelVelocity* velocity = command->mutable_move_command()->mutable_wheel_velocity();
        velocity->set_front_right(frontRightWheelVelocity);
        velocity->set_back_right(backRightWheelVelocity);
        velocity->set_back_left(backLeftWheelVelocity);
        velocity->set_front_left(frontLeftWheelVelocity);
    }
    void RobotControlCommand::addRobotControlWithLocalSpeeds(
                int robotId,
                float kickSpeed,
                float kickAngle,
                float dribblerSpeed,
                float forwardVelocity,
                float leftVelocity,
                float angularVelocity) {
        proto::RobotCommand* command = this->controlCommand.add_robot_commands();
        command->set_id(robotId);
        command->set_kick_speed(kickSpeed);
        command->set_kick_angle(kickAngle);
        command->set_dribbler_speed(dribblerSpeed);

        proto::MoveLocalVelocity* velocity = command->mutable_move_command()->mutable_local_velocity();
        velocity->set_forward(forwardVelocity);
        velocity->set_left(leftVelocity);
        velocity->set_angular(angularVelocity);
    }
    void RobotControlCommand::addRobotControlWithGlobalSpeeds(
                int robotId,
                float kickSpeed,
                float kickAngle,
                float dribblerSpeed,
                float xVelocity,
                float yVelocity,
                float angularVelocity) {
        proto::RobotCommand* command = this->controlCommand.add_robot_commands();
        command->set_id(robotId);
        command->set_kick_speed(kickSpeed);
        command->set_kick_angle(kickAngle);
        command->set_dribbler_speed(dribblerSpeed);

        proto::MoveGlobalVelocity* velocity = command->mutable_move_command()->mutable_global_velocity();
        velocity->set_x(xVelocity);
        velocity->set_y(yVelocity);
        velocity->set_angular(angularVelocity);
    }
    proto::RobotControl& RobotControlCommand::getPacket() {
        return this->controlCommand;
    }
}