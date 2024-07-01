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
#include <array>

namespace math {
    namespace armDynamics {

        enum DYNAMICS_SOLVE_TYPE{
            GCOMP,
            SIMPLE_PD,
            COMPUTED,
        };

        Eigen::Vector3d gravity(0, 0, -9.81);

        // JointState => thetalist, thetaDotlist, thetaDoubleDotlist
        // torqueList

        using torqueList = Eigen::VectorXd;

    class armDynamicsSolver : public Solver<std::vector<param::JointState>, torqueList, std::vector<param::LinkParam>> {

        public:

        armDynamicsSolver(std::vector<param::LinkParam> inputLinkParams,
                          std::vector<param::DHParam> inputDHParams,
                          std::vector<Eigen::VectorXd> inputA,
                          std::shared_ptr<std::vector<param::JointState>> jointStatesPtr,
                          std::shared_ptr<Eigen::VectorXd> torquesPtr);

        armDynamicsSolver();

        // Should Free Variables
            ~armDynamicsSolver();
            void setSolverType(DYNAMICS_SOLVE_TYPE solverType);
            void solve() override;
            void getJointState();
            void setTorques();
            void updateTransList(std::vector<Transform> transList);


        private:

            //* Fixed Variables
            std::vector<param::LinkParam> linkParams_;
            std::vector<param::DHParam> dhParams_;
            std::vector<Eigen::VectorXd> A_;
            Eigen::MatrixXd AMatrix_;
            Eigne::MatrixXd GMatrix_;
            std::vector<Transform> transList_;
            Eigen::MatrixXd jacobian_;
            std::vector<Eigen::VectorXd> twistList_;


            //* Shared Variables
            std::shared_ptr<std::vector<param::JointState>> jointStatesPtr_;
            std::shared_ptr<Eigen::VectorXd> torquesPtr_;
            std::vector<param::JointState> inputJointState_;
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
            Eigen::VectorXd calculateTorques();
            Eigen::VectorXd calculateOnlyGComp();
            void calculateJacobian();
            void calculateTwist();



            inline Eigen::MatrixXd makeLMatrix(const std::vector<Transform>& transList);
            inline Eigen::MatrixXd makeWMatrix(const std::vector<Transform>& transList);
            inline Eigen::MatrixXd makeAdAtheta(const std::vector<Eigen::VectorXd> AthetaList);
            inline Eigen::MatrixXd makeAdV(const std::vector<Eigen::VectorXd> TwistList);






        };
    }
}

#endif //VR_ARM_ARMDYNAMICSSOLVER_HPP