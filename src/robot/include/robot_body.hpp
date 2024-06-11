//
// Created by gj on 24. 3. 5.
//

#ifndef VR_ARM_ROBOT_BODY_HPP
#define VR_ARM_ROBOT_BODY_HPP

#include <memory>
#include <array>
#include <Eigen/Dense>

#include "fsm.hpp"
#include "math/armDynamcisSolver.hpp"
#include "math/armKinematicsSolver.hpp"


namespace robot {
    class Robot {
    public:
        Robot() = default;

    private:

        fsm::FiniteStateMachine fsm_;
        Eigen::VectorXd angle_;
        Eigen::VectorXd EETransform_;

        std::unique_ptr<math::armDynamics::armDynamicsSolver> dynamicsSolver_{nullptr};
        std::unique_ptr<math::armKinematics::armKinematicsSolver> kinematicsSolver_{nullptr};

    };
}
#endif //ROBOT_BODY_HPP
