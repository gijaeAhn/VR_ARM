//
// Created by gj on 24. 3. 4.
//

#include "driver.hpp"



namespace rmd_driver{


    Driver::Driver(std::string const& ifname)
            : Node{ifname} {
        return;
    }


    void Driver::addMotor(std::uint32_t actuator_id){
        updateIds(actuator_id);
    }

    void Driver::MotorRunning(std::uint32_t actuator_id){
        MotorRunningRequest const request {};
        [[maybe_unused]] auto const response {sendRecv<MotorRunningResponse>(actuator_id,request)};
        return;
    }


    Feedback Driver::sendTorqueSetpoint(std::uint32_t actuator_id,float const current) {
        SetTorqueRequest const request {current};
        auto const response {sendRecv<SetTorqueResponse>(actuator_id,request)};
        return response.getStatus();
    }

    void Driver::shutdownMotor(std::uint32_t actuator_id) {
        ShutdownMotorRequest const request {};
        [[maybe_unused]] auto const response {sendRecv<ShutdownMotorResponse>(actuator_id,request)};
        return;
    }

    void Driver::stopMotor(std::uint32_t actuator_id) {
        StopMotorRequest const request {};
        [[maybe_unused]] auto const response {sendRecv<StopMotorResponse>(actuator_id,request)};
        return;
    }


}