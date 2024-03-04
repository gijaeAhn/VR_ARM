//
// Created by gj on 24. 3. 4.
//

#include <cstdint>
#include <cstring>
#include <string>

#include "protocol/request.hpp"
#include "protocol/single_motor_message.hpp"
#include "exceptions.hpp"

namespace rmd_driver{

    SetTorqueRequest::SetTorqueRequest(float const current)
            : SingleMotorRequest{} {
        if ((current < -20.0f) || (current > 20.0f)) {
            throw ValueRangeException("Current value '" + std::to_string(current) + "' out of range [-20.0, 20.0]");
        }
        auto const c {static_cast<std::int16_t>(current/0.01f)};
        setAt(c, 4);
        return;
    }

    float SetTorqueRequest::getTorqueCurrent() const noexcept {
    return static_cast<float>(getAs<std::int16_t>(4))*0.01f;}



}



