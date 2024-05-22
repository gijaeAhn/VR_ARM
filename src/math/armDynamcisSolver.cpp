//
// Created by gj on 4/29/24.
//
#include "armDynamcisSolver.hpp"

namespace math{
    namespace armDynamics{
        armDynamicsSolver::armDynamicsSolver() {
            dof_ = 6;
            inputJS_.resize(dof_+1);
            inputError_.resize(3);
            solution_.resize(dof_);
            massMatrix_.resize(dof_,dof_);
            lMatrix_.resize(6*dof_,6*dof_);
            coriolis_.resize(dof_);
            gravityComp_.resize(dof_);

            spatialInertialMatrixList_.resize(dof_);
            spatialInertialMatrix_.resize(6*dof_,6*dof_);
            screwAxisList_.resize(dof_);
            screwAxisMatrix_.resize(6*dof_,6*dof_);
            // Last Transform for EE
            transformList_.resize(dof_ +1 );
            // For subDiagonal Transform Matrix
            relativeTransList_.resize(dof_-1);
            for(uint8_t index =0 ; index < dof_; index++){
                relativeTransList_[index].resize(dof_-1-index);
            }

            // Base Frame to 6th link Frame
            comList_.resize(dof_+1);
            Transform transInit;
            transInit.clear();

            for(uint8_t index = 0; index < dof_ ; index++){
                transformList_[index] = transInit;
            }
            for(uint8_t index = 0; index < dof_ ; index++){
                comList_[index] = transInit;
            }




        }
        armDynamicsSolver::~armDynamicsSolver() {}


        void armDynamicsSolver::getInput(math::armDynamics::jointState &&inputJS) {
            inputJS_ = inputJS;
        }

        void armDynamicsSolver::getError(Eigen::VectorXd error) {
            inputError_ = error;
        }

        void armDynamicsSolver::getScrewAxis(Eigen::VectorXd screwAxis, uint8_t index) {
            screwAxisList_[index] = screwAxis;
        }

        void armDynamicsSolver::getScrewAxis(std::vector<Eigen::VectorXd> sal) {
            screwAxisList_ = sal;
        }

        void armDynamicsSolver::getSpatialInertialMatrix(Eigen::MatrixXd sim, uint8_t index) {
            spatialInertialMatrixList_[index] = sim;
        }

        void armDynamicsSolver::getSpatialInertialMatrix(std::vector<Eigen::MatrixXd> matrixList){
            spatialInertialMatrixList_ = matrixList;
        }

        void armDynamicsSolver::updateComList(std::vector<Transform> inputList) {
            for(uint8_t index =0; index  <dof_; index++){
                comList_[index] = inputList[index];
            }
        }

        void armDynamicsSolver::updateTransform() {
            for(uint8_t index = 0; index < dof_ + 1; index++){
                if(index == 0){
                    continue;
                }else {
                    Transform dummy2;
                    dummy2.clear();
                    //TransformList[1] = T12
                    // Screw axis should start from 00
                    Eigen::Matrix3d tempSo3 = VecToso3(screwAxisList_[index].head<3>());
                    Eigen::Vector3d tempVec = screwAxisList_[index].segment<3>(3);
                    //Input JS should start from joint0 which is 0
                    //To match the index
                    //If index = 6 we are calculating T6EE
                    transformList_[index] = inv(comList_[index]) * comList_[index + 1] * ExpTransA(tempSo3, tempVec, inputJS_[index][0]);
                }
            }


            // WIP
            //Mulitply from Matrix i to Matrix j
            for(uint8_t i = 0; i < relativeTransList_.size(); i++){
                for(uint8_t j = i; j < relativeTransList_[i].size(); j++){
                    Transform dummy;
                    dummy.clear();
                    //Range transform 01 ~ 6EE
                    for(uint8_t k = i +1; k < j + 2; k++){
                        dummy = dummy * transformList_[k];
                    }
                    relativeTransList_[i][j] = dummy;
                }

            }

        }


        void armDynamicsSolver::calculateMassMatrix() {
            Eigen::MatrixXd inverseScrewMatrix = screwAxisMatrix_.inverse();
            Eigen::MatrixXd inverseLMatrix = lMatrix_.inverse();
            massMatrix_ = inverseScrewMatrix     *
                          inverseLMatrix         *
                          spatialInertialMatrix_ *
                          lMatrix_               *
                          screwAxisMatrix_       ;
        }

        void armDynamicsSolver::calculateLMatrix() {
            //insert identity matrix diagonally <6,6>

        }





    }
}