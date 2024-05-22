//
// Created by gj on 4/30/24.
//

#ifndef VR_ARM_ARMKINEMATICSSOLVER_HPP
#define VR_ARM_ARMKINEMATICSSOLVER_HPP

#include "Solver.hpp"
#include "transform.hpp"
#include <Eigen/Dense>

#include <math.h>
#include "rigidBody.hpp"

namespace math {
    namespace armKinematics {

        using jointList = Eigen::VectorXd;
        using paramList = std::vector<Eigen::VectorXd>;

        class armKinematicsSolver : public Solver<Transform, jointList, paramList> {

        public:

            armKinematicsSolver();
            ~armKinematicsSolver();

            void solve();
            void apply(jointList& cp);
            void getInput(Transform&& inputTrans);

        private:
            double shoulderHeight;
            double upperArmLength;
            double lowerArmLength;
            double wrist1x;
            double wrist1z;
            double wrist2x;
            double wrist2z;


            Transform inputT_;
            jointList solution_;
        };
    }
}

#endif //VR_ARM_ARMKINEMATICSSOLVER_HPP
