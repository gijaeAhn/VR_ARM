//
// Created by gj on 4/30/24.
//

#include "armKinematicsSolver.hpp"



namespace math{
    namespace armKinematics{

        armKinematicsSolver::armKinematicsSolver(std::shared_ptr<Transform> inputTransformPtr,
                                                 std::shared_ptr<std::vector<param::JointState>> inputJointStatePtr,
                                                 std::vector<param::DHParam>& DHParam) {
            //temporary value
//
            //DH Param to lenghts
            inputTransformPtr_ = inputTransformPtr;
            jointStatePtr_ = inputJointStatePtr;
            //DH Parma to arm lengths
            dhParams_ = DHParam;
        }

        armKinematicsSolver::~armKinematicsSolver() {

        }

        void armKinematicsSolver::getInput() {
            inputT_  = *inputTransformPtr_;
        }

        void armKinematicsSolver::setJointState() {
            *jointStatePtr_ = jointStateQueue_.front();
        }

        void armKinematicsSolver::apply(math::armKinematics::jointList &cp) {
            cp = solution_;
        }

        void armKinematicsSolver::solve() {
//            Eigen::Matrix3d inputRotation =  inputT_.t.block<3,3>(0,0);
            Eigen::Vector3d inputTranslation = inputT_.t.block<3,1>(0,3);

            //Frist get Joint1
            double joint1 = std::atan2(inputTranslation(2),inputTranslation(1));

            //Making T1
            Transform T1 ;
            T1.clear();
            T1.rotateZ(joint1).translateZ(shoulderHeight);

            // Get Joint6
            // If Joint5 is -90 or 90 => singularity
            Transform tempInput1 = inputT_; // Should be copy constructor
            Transform T16 = inv(T1) * tempInput1;
            double joint6 = 0;
            if (!math::NearZero(T16(1,0))) {
                joint6 = std::atan(T16(1, 0) / T16(1, 1));
            }
            else {
                // Previous joint
            }

            //Get Joint 5
            // Sign?
            double joint5 = -std::asin(T16(1,3)/wrist2z);

            //Get joint 4
            Transform tempInput2 = inputT_;
            // tempInput2 now T05
            // T5 x,y,z axis = T0 -z,y,x
            tempInput2.rotateZ(-joint6).translateZ(-wrist2z).rotateY(PI/2).translateZ(-wrist2x);
            Transform T15 = inv(T1) * tempInput2;
            Transform T15dummy = T15;
            Transform T14 = T15dummy.translateX(wrist1z).translateZ(wrist1x).rotateX(-PI/2).rotateZ(PI);
            Transform T14dummy = T14;
            //Refactor Joint4 direction (Should be inversed)
            // T4 x,y,z axis = T0  z,x,y
            double joint4 = std::atan2(T14(2,1),T14(2,2));

            //Get joint3 and joint2
            // tempInput2 now T04
            double tempLength   =         std::sqrt(std::pow(T14dummy(0,3),2) +
                                                    std::pow(T14dummy(1,3),2) +
                                                    std::pow(T14dummy(2,3),2));
            double joint3       = 2*PI - std::acos((std::pow(upperArmLength,2) +
                                                    std::pow(lowerArmLength,2) -
                                                    std::pow(tempLength,2))    /
                                                    (2*upperArmLength*lowerArmLength));

            double tempAngle1 = std::acos(std::sqrt(std::pow(T14dummy(0,3),2)+ std::pow(T14dummy(1,3),2))/tempLength);

            double tempAngle2 = std::acos((std::pow(tempLength,2)+ std::pow(upperArmLength,2)-std::pow(lowerArmLength,2))/
                                             (2 * tempLength * upperArmLength));

            double joint2 = -(PI/2 - tempAngle1 - tempAngle2);
            solution_(0) = joint1;
            solution_(1) = joint2;
            solution_(2) = joint3;
            solution_(3) = joint4;
            solution_(4) = joint5;
            solution_(5) = joint6;

            updateJointStates();
        }

        void armKinematicsSolver::updateJointStates() {
            auto now = std::chrono::steady_clock::now();
            auto temp = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime_);
            double dt = static_cast<double>(temp.count()) / 1000.0;
            lastTime_ = now;
            std::vector<param::JointState> tempJointStateList(dof_);

            // Update jointStates_ with calculated velocities and accelerations
            // Assuming the joint positions are already updated in jointStates_
            if (!jointStateQueue_.empty()) {
                const auto& previousJointStates = jointStateQueue_.front();

                for (size_t i = 0; i < dof_; ++i) {
                    tempJointStateList[i].positionAngle     = solution_[static_cast<long>(i)];
                    tempJointStateList[i].velocityAngle     =(solution_[static_cast<long>(i)] - previousJointStates[i].positionAngle) / dt;
                    tempJointStateList[i].accelerationAngle =(tempJointStateList[i].velocityAngle - previousJointStates[i].velocityAngle) / dt;
                }

                // Update the jointStateQueue_ with the new joint states
                jointStateQueue_.push(tempJointStateList);

                // Optional: Maintain a fixed size for the queue (e.g., keeping the last N states)
                if (jointStateQueue_.size() > MAX_QUEUE_SIZE) {
                    jointStateQueue_.pop();
                }
            } else {
                // If the queue is empty, initialize it with the first set of joint states
                for (size_t i = 0; i < dof_; ++i) {
                    tempJointStateList[i].positionAngle = solution_[static_cast<long>(i)];
                    tempJointStateList[i].velocityAngle = 0.0;
                    tempJointStateList[i].accelerationAngle = 0.0;
                }
                jointStateQueue_.push(tempJointStateList);
            }

        }


    }



}
