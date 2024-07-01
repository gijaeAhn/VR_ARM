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
            // Make A Matrix
            for(size_t i = 0; i < dof_; i++){
               AMatrix_.block(6*i,i,6,1) = A_[i];
            }


        }

        armDynamicsSolver::~armDynamicsSolver() {
        }

        void armDynamicsSolver::getJointState(){
            inputJointState_ = *jointStatesPtr_;
        }

        void armDynamicsSolver::setTorques() {
            *torquesPtr_ = solutionTorque_;
        }

        void armDynamicsSolver::updateTransList(std::vector<Transform> transList) {
            transList_ = transList;
        }

        void armDynamicsSolver::calculateJacobian() {

        }

        void armDynamicsSolver::calculateTwist() {
            Eigen::VectorXd thetaDot;
            thetaDot.resize(dof_);
            for(size_t index = 0; index < dof_; index++){
                thetaDot[index] = inputJointState_[index][1];
            }
            //Is this 6by6 * 6by 1 or 1by6
             twistList_ = jacobian_ * thetaDot;
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

        inline Eigen::MatrixXd armDynamicsSolver::makeLMatrix(const std::vector<Transform>& transList) {
            Eigen::MatrixXd result = Eigen::MatrixXd::Identity(36,36);
            for (std::size_t i = 0; i < 6; i++) {
                for (std::size_t j = i; j < 6; j++) {
                    Eigen::MatrixXd dummy = Eigen::MatrixXd::Identity(36,36);
                    for (std::size_t k = i; k <= j; k++) {
                        if (k == i) {
                        } else {
                            dummy = dummy * transList[k].t;
                        }
                    }
                    Eigen::MatrixXd dummyDummy = Adjoint(dummy);
                    result.block<6, 6>(6*j, 6*i) = dummyDummy;
                }
            }
            return result;
        }

        inline Eigen::MatrixXd armDynamicsSolver::makeWMatrix(const std::vector <Transform> &transList) {
            Eigen::MatrixXd result = Eigen::MatrixXd::Identity(36,36);
        }


        inline Eigen::MatrixXd armDynamicsSolver::makeAdAtheta(const std::vector <Eigen::VectorXd> AthetaList) {
            Eigen::MatrixXd result;
            for(size_t index =0; index < dof_; index++){
                result.block<6,6>(6 * index,6 * index) = ad(AthetaList[index] * inputJointState_[index][1]);
            }
            return result;
        }

        inline Eigen::MatrixXd armDynamicsSolver::makeAdV(const std::vector <Eigen::VectorXd> TwistList) {
            Eigen::MatrixXd result;
            for(size_t index = 0; index < dof_; index++){
                result.block<6,6>(6*index, 6*index) = ad(TwistList[index]);
            }
            return result;
        }


        Eigen::VectorXd armDynamicsSolver::calculateTorques() {
            Eigen::MatrixXd lMatrix   = makeLMatrix(transList_);
            Eigen::MatrixXd adATheta  = makeAdAtheta(A_);
            Eigen::MatrixXd wMatrix   = makeWMatrix(transList_);
            Eigen::MatrixXd adVMatrix = makeAdV(twistList_);


            Eigen::VectorXd thetaDot;
            thetaDot.resize(dof_);
            for(size_t index = 0; index < dof_; index ++){
                thetaDot[index] = inputJointState_[index][1];
            }

            Eigen::VectorXd thetaDoubleDot;
            thetaDoubleDot.resize(dof_);
            for(size_t index = 0; index < dof_; index ++){
                thetaDot[index] = inputJointState_[index][2];
            }



            Eigen::MatrixXd mMatrix = AMatrix_.transpose()          *
                                      lMatrix.transpose()           *
                                      GMatrix_                      *
                                      lMatrix                       *
                                      AMatrix_;

            Eigen::MatrixXd cMatrix = AMatrix_.transpose()          *
                                      lMatrix.transpose()           *
                                      (GMatrix_                     *
                                       lMatrix                      *
                                       adATheta                     *
                                       wMatrix                      +
                                       adV.transpose()              *
                                       GMatrix_)                    *
                                       lMatrix                      *
                                       ( AMatrix_ * thetaDot );

            Eigen::VectorXd gCompTorque = calculateOnlyGComp();

            return mMatrix * thetaDoubleDot + cMatrix * thetaDot + gCompTorque;
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
