//
// Created by gj on 4/30/24.
//

#ifndef VR_ARM_ARMKINEMATICSSOLVER_HPP
#define VR_ARM_ARMKINEMATICSSOLVER_HPP

#include "Solver.hpp"
#include "transform.hpp"
#include "rigidBody.hpp"

#include <Eigen/Dense>
#include <math.h>
#include <chrono>
#include <queue>
#include <memory>


#include "utilities/include/param.hpp"



namespace math {
    namespace armKinematics {

        using jointList = Eigen::VectorXd;


    class armKinematicsSolver : public Solver<Transform, jointList, std::vector<param::DHParam>> {

        public:

            armKinematicsSolver(std::shared_ptr<Transform> inputTransformPtr,
                                std::shared_ptr<std::vector<param::JointState>> inputJointStatePtr,
                                std::vector<param::DHParam>& DHParam);
            ~armKinematicsSolver();

            void solve() override;
            void apply(jointList& cp) override;
            void getInput() override;
            void setJointState();




        private:
            double shoulderHeight;
            double upperArmLength;
            double lowerArmLength;
            double wrist1x;
            double wrist1z;
            double wrist2x;
            double wrist2z;


            //Sharing Variables
            std::shared_ptr<Transform> inputTransformPtr_;
            std::shared_ptr<std::vector<param::JointState>> jointStatePtr_;

            Transform inputT_;
            jointList solution_;
            std::queue<std::vector<param::JointState>> jointStateQueue_;
            std::vector<param::DHParam> dhParams_;

            std::chrono::time_point<std::chrono::steady_clock> lastTime_;

            //Fixed Internal Params
            size_t dof_ = DOF;

            //Internal Functions
            void updateJointStates();




        };
    }
}

#endif //VR_ARM_ARMKINEMATICSSOLVER_HPP
