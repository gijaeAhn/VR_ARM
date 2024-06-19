//
// Created by GJ on 6/13/24.
//

#ifndef VR_ARM_PARAM_HPP
#define VR_ARM_PARAM_HPP

#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>


#define  DOF                           6
#define  ROBOT_MOTOR_SIZE              4
#define  RMD_MOTOR_SIZE                2
#define  DYNAMIXEL_MOTOR_SIZE          2

#define  MOTORINIT_TIME                0.1f


#define IK_THREAD_CYCLE                10 //ms
#define DYNAMICS_THREAD_CYCLE          10 //ms
#define ZMQ_THREAD_CYCLE               10 //ms

#define MAX_QUEUE_SIZE                 10

#ifdef USE_DOUBLE_PRECISION
#define ZMQ_PRECISION_TYPE double
#else
#define ZMQ_PRECISION_TYPE float
#endif

namespace param {
    struct DHParam {
        double alpha, a, d, theta;
    };

    struct LinkParam {
        double mass;
        Eigen::Vector3d centerOfMass;
        Eigen::Matrix3d inertiaTensor;
    };

    struct JointState {
        ZMQ_PRECISION_TYPE1 positionAngle;
        ZMQ_PRECISION_TYPE1 velocityAngle;
        ZMQ_PRECISION_TYPE1 accelerationAngle;
    };

    struct ArmConfigParam{
        std::map<std::string, LinkParam> links;
        std::map<std::string, DHParam> joints;
    };

}













#endif //VR_ARM_PARAM_HPP
