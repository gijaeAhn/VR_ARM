//
// Created by gj on 24. 3. 4.
//

#ifndef RMD_DRIVER_MOTORPARA_HPP
#define RMD_DRIVER_MOTORPARA_HPP
#pragma once

namespace rmd_driver{

    const int oneShaftCycle = 40;
    const int maxShaftAngle = 65535;
    const int gearRatio = 9;
    const float max_current = 2.125;
    const float max_pid_current = 1.125;


//not using it right now.
//Should be updated non linearly
    const int X8_CONST     = 51;
    const int X8_V2_CONST  = 45;
    const int X6_CONST     = 75;

    const int X8_SHAFTCYCLE    = 60;
    const int X8_V2_SHAFTCYCLE  = 60;
    const int X6_SHAFTCYCLE     = 45;

    const int X8_GEAR     = 6;
    const int X8_V2_GEAR  = 6;
    const int X6_GEAR     = 6 ;

}

#endif //RMD_DRIVER_MOTORPARA_HPP
