//
// Created by gj on 4/29/24.
//

#ifndef VR_ARM_ARMDYNAMICSSOLVER_HPP
#define VR_ARM_ARMDYNAMICSSOLVER_HPP

#include "Solver.hpp"
#include "transform.hpp"
#include "rigidBody.hpp"

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace math {
    namespace armDynamics {

        // JointState => thetalist, thetaDotlist, thetaDoubleDotlist
        // torqueList

        using torqueList = Eigen::VectorXd;

    class armDynamicsSolver : public Solver<std::vector<JointState>, torqueList, std::vector<LinkParam>> {

        public:

        armDynamicsSolver(std::vector<LinkParam> inputLinkParams,
                          std::vector<DHParam> inputDHParams,
                          std::vector<Eigen::VectorXd> inputA,
                          std::shared_ptr<std::vector<JointState>> jointStatesPtr,
                          std::shared_ptr<Eigen::VectorXd> torquesPtr);

        armDynamicsSolver();

        // Should Free Variables
            ~armDynamicsSolver();
            void setSolverType(DYNAMICS_SOLVE_TYPE solverType);
            void solve() override;
            void getJointState();
            void setTorques();


        private:

            //* Fixed Variables
            std::vector<LinkParam> linkParams_;
            std::vector<DHParam> dhParams_;
            std::vector<Eigen::VectorXd> A_;


            //* Shared Variables
            std::shared_ptr<std::vector<JointState>> jointStatesPtr_;
            std::shared_ptr<Eigen::VectorXd> torquesPtr_;
            std::vector<JointState> inputJointState_;
            Eigen::VectorXd solutionTorque_;

            //* State Variables
            DYNAMICS_SOLVE_TYPE solverType_;



            uint8_t dof_= 6;
//            double shoulderHeight;
//            double upperArmLength;
//            double lowerArmLength;
//            double wrist1x;
//            double wrist1z;
//            double wrist2x;
//            double wrist2z;
            //*//

            //* Functions
            void calculateTorques();
            void calculateOnlyGComp();






        };
    }
}

#endif //VR_ARM_ARMDYNAMICSSOLVER_HPP