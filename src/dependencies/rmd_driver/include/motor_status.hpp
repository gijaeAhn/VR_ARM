//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_MOTOR_STATUS_HPP
#define RMD_DRIVER_MOTOR_STATUS_HPP
#pragma once


namespace rmd_driver{


    class MotorStatus {
        public:
    /**\fn MotorStatus
       * \brief
       *    Class constructor
       *
       * \param[in] temperature_
       *    The temperature of the actuator in degree Celsius with a resolution of 1 deg C
       * \param[in] current_
       *    The current currently used by the actuator in Ampere with a resolution of 0.01A
       * \param[in] shaft_speed_
       *    The output shaft velocity in degree per second with a resolution of 1dps
       * \param[in] shaft_angle_
       *    The output shaft angle in degrees with a resolution of 1 deg and a maximum range of 32767
    */

        MotorStatus(const int temperature_ =0, const float current_ = 0.0f, const float shaft_speed_ = 0.0f, const float shaft_angle_ = 0.0f) noexcept;
        MotorStatus(MotorStatus const&) = default;
        MotorStatus& operator = (MotorStatus const&) = default;
        MotorStatus(MotorStatus&&) = default;
        MotorStatus& operator = (MotorStatus&&) = default;


        protected :

        int temperature;
        float current;
        float shaft_speed;
        float shaft_angle;
    };


    MotorStatus::MotorStatus(const int temperature_, const float current_ , const float shaft_speed_, const float shaft_angle_ ) noexcept
    : temperature{temperature_}, current{current_}, shaft_speed{shaft_speed_}, shaft_angle{shaft_angle_} {
        return;
    }

}

#endif //RMD_DRIVER_MOTOR_STATUS_HPP
