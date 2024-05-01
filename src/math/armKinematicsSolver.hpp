//
// Created by gj on 4/30/24.
//

#ifndef VR_ARM_ARMKINEMATICSSOLVER_HPP
#define VR_ARM_ARMKINEMATICSSOLVER_HPP

#include "Solver.hpp"
#include "transform.hpp"
#include "eigen3/Eigen/Dense"

namespace math {
    namespace armKinematics {

        using jointList = Eigen::VectorXd;
        using parameter = Eigen::MatrixXd;

        class armKinematicsSolver : public Solver<Transform, jointList> {

        public:

            armKinematicsSolver();
            ~armKinematicsSolver();

            void solve();
            void apply(jointList& cp);
            void getInput(Transform&& inputTrans);

        private:

            parameter param_;
            Transform inputT_;
            jointList solution_;
        };
    }
}

#endif //VR_ARM_ARMKINEMATICSSOLVER_HPP
