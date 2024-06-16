//
// Created by gj on 4/29/24.
//
#include "armDynamcisSolver.hpp"

#include <utility>

namespace math {
    namespace armDynamics {



        armDynamicsSolver::armDynamicsSolver(std::vector<param::LinkParam> inputLinkParams,
                                             std::vector<param::DHParam> inputDHParams,
                                             std::vector<Eigen::VectorXd> inputA,
                                             std::shared_ptr<std::vector<param::JointState>> jointStatesPtr,
                                             std::shared_ptr<Eigen::VectorXd> torquesPtr)
                : linkParams_(std::move(inputLinkParams)),
                  dhParams_(std::move(inputDHParams)),
                  A_(std::move(inputA)),
                  jointStatesPtr_(jointStatesPtr),
                  torquesPtr_(torquesPtr),
                  dof_(6) {
        }

        armDynamicsSolver::~armDynamicsSolver() {
        }

        void armDynamicsSolver::getJointState(){
            inputJointState_ = *jointStatesPtr_;
        }

        void armDynamicsSolver::setTorques() {
            *torquesPtr_ = solutionTorque_;
        }


        void armDynamicsSolver::solve() {
            getJointState();

            switch (solverType_) {
                case GCOMP:
                    calculateOnlyGComp();
                    break;
                case SIMPLE_PD:
                    // Modern cpp printf out function
                    // Not implemented yet
                    break;
                case COMPUTED:
                    calculateTorques();
                    break;
                default:
                    throw std::invalid_argument("Invalid dynamics solve type");
            }

            setTorques();

        }

        void armDynamicsSolver::calculateTorques() {
            std::vector<Eigen::Matrix4d> T(dof_);
            std::vector<Eigen::VectorXd> V(dof_, Eigen::VectorXd::Zero(6));
            std::vector<Eigen::VectorXd> Vdot(dof_, Eigen::VectorXd::Zero(6));
            //F is Spatial Force so that it has 6 dimensions
            std::vector<Eigen::VectorXd> F(dof_, Eigen::VectorXd::Zero(6));
            Eigen::VectorXd torques(dof_, 0);

            // Forward Recursion: Compute position, velocity, and acceleration
            for (size_t index = 0; index < dof_; ++index) {
                if (index == 0) {

                    T[index]    = computeTransformationMatrix(dhParams_[index], inputJointState_[index].positionAngle);

                    V[index]    = Eigen::VectorXd::Zero(dof_);

                    Vdot[index] = Eigen::VectorXd::Zero(dof_);

                    Vdot[index].tail(3) = -gravity;

                } else {

                    T[index]     =  T[index - 1] *
                                    computeTransformationMatrix(dhParams_[index], inputJointState_[index].positionAngle);

                    V[index]     =  Adjoint(T[index - 1]) * V[index - 1] + A_[index] * inputJointState_[index].velocityAngle;

                    Vdot[index]  =  Adjoint(T[index - 1]) * Vdot[index - 1] +
                                    ad(V[index]) * A_[index] * inputJointState_[index].velocityAngle +
                                    A_[index] * inputJointState_[index].accelerationAngle;
                }
            }

            // Backward Recursion: Compute forces and torques
            for (size_t index = dof_ - 1; index --> 0;) {
                if (index == static_cast<size_t>(dof_-1)) {
                    F[index] = linkParams_[index].inertiaTensor * Vdot[index] -
                               ad(V[index]).transpose() * (linkParams_[index].inertiaTensor * V[index]);
                }
                else {
                    F[index] = Adjoint(T[index + 1].transpose()) * F[index + 1] +
                               linkParams_[index].inertiaTensor * Vdot[index] -
                               ad(V[index]).transpose() * (linkParams_[index].inertiaTensor * V[index]);
                }
                torques[static_cast<long>(index)] = F[index].transpose() * A_[index];
            }
            solutionTorque_ = torques;
        }

        void armDynamicsSolver::calculateOnlyGComp() {
            std::vector<Eigen::Matrix4d> T(dof_);
            std::vector<Eigen::VectorXd> V(dof_, Eigen::VectorXd::Zero(6));
            std::vector<Eigen::VectorXd> F(dof_, Eigen::VectorXd::Zero(6));
            Eigen::VectorXd torques(dof_, 0);

            // Forward Recursion: Compute position and initialize velocity and acceleration
            for (size_t index = 0; index < dof_; ++index) {
                if (index == 0) {
                    T[index] = computeTransformationMatrix(dhParams_[index], inputJointState_[index].positionAngle);
                    V[index] = Eigen::VectorXd::Zero(6);
                    V[index].tail(3) = -gravity;
                } else {
                    T[index] = T[index - 1] * computeTransformationMatrix(dhParams_[index], inputJointState_[index].positionAngle);
                    V[index] = Adjoint(T[index - 1]) * V[index - 1];
                }
            }

            // Backward Recursion: Compute gravitational forces and torques
            for (size_t index = dof_ - 1; index-- >0;) {
                if (index == static_cast<size_t>(dof_-1)) {
                    F[index] = linkParams_[index].inertiaTensor * V[index];
                } else {
                    F[index] = Adjoint(T[index + 1].transpose()) * F[index + 1] + linkParams_[index].inertiaTensor * V[index];
                }
                torques[static_cast<long>(index)] = F[index].transpose() * A_[index];
            }
            solutionTorque_ = torques;
        }
        

    }

}
