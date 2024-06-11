//
// Created by gj on 4/29/24.
//
#include "armDynamcisSolver.hpp"

namespace math {
    namespace armDynamics {

        armDynamicsSolver::armDynamicsSolver() {

        }

        armDynamicsSolver::~armDynamicsSolver() {}

        void armDynamicsSolver::getDhParam(const vector<math::DHParam> &inputDhList) {
            dhParams_ = inputDhList;
        }

        void armDynamicsSolver::getLinkParam(const vector<LinkParam>& inputLinkParam) {
            linkParams_ = inputLinkParam;
        }

        void armDynamicsSolver::getJointState(const vector<JointState>& inputJointState) {
            inputJointState_ = inputJointState;
        }


        void armDynamicsSolver::solve() {

        }

        void armDynamicsSolver::calculateTorques() {
            vector<Matrix4d> T(dof_);
            vector<VectorXd> V(dof_, VectorXd::Zero(6));
            vector<VectorXd> Vdot(dof_, VectorXd::Zero(6));
            //F is Spatial Force so that it has 6 dimensions
            vector<VectorXd> F(dof_, VectorXd::Zero(6));
            vector<double> torques(dof_, 0);

            // Forward Recursion: Compute position, velocity, and acceleration
            for (uint8_t index = 0; index < dof_; ++index) {
                if (index == 0) {
                    T[index] = computeTransformationMatrix(dhParams_[index], inputJointState_[index].positionAngle);
                    V[index] = VectorXd::Zero(6);
                    Vdot[index] = VectorXd::Zero(6);
                    Vdot[index].tail(3) = -gravity;
                } else {
                    T[index] = T[index - 1] *
                               computeTransformationMatrix(dhParams_[index], inputJointState_[index].positionAngle);
                    V[index] = Adjoint(T[index - 1]) * V[index - 1] + A_[index] * inputJointState_[index].velocityAngle;
                    Vdot[index] = Adjoint(T[index - 1]) * Vdot[index - 1] +
                                  ad(V[index]) * A_[index] * inputJointState_[index].velocityAngle +
                                  A_[index] * inputJointState_[index].accelerationAngle;
                }
            }

            // Backward Recursion: Compute forces and torques
            for (uint8_t index = dof_ - 1; index >= 0; --index) {
                if (index == dof_ - 1)
                    F[index] = linkParams_[index].inertiaTensor * Vdot[index] -
                               ad(V[index]).transpose() * (linkParams_[index].inertiaTensor * V[index]);
                else {
                    F[index] = Adjoint(T[index + 1].transpose()) * F[index + 1] +
                               linkParams_[index].inertiaTensor * Vdot[index] -
                               ad(V[index]).transpose() * (linkParams_[index].inertiaTensor * V[index]);
                }
                torques[index] = F[index].transpose() * A_[index];
            }

            solutionTorque_ = torques;
        }
    }

}
