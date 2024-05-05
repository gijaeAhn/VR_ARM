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

        void armDynamicsSolver::getSpatialInertialMatrix(std::vector<Eigen::MatrixXd> matrixlist){
            spatialInertialMatrixList_ = matrixlist;
        }

        void armDynamicsSolver::updateComList(std::vector<Transform> inputlist) {
            for(uint8_t index =0; index  <dof_; index++){
                comList_[index] = inputlist[index];
            }
        }

        void armDynamicsSolver::updateTransform() {
            for(uint8_t index = 0; index < dof_ + 1; index++){
                if(index == 0){
                    Transform dummy1;
                    dummy1.clear();
                    // Should be modifed == base to frame 1 (Link1 COM>
                    transformList_[index] = comList_[0] * dummy1;
                }else {
                    Transform dummy2;
                    dummy2.clear();
                    //TransformList[1] = T12
                    // Screw axis should start from 00
                    Eigen::Matrix3d tempSo3 = VecToso3(screwAxisList_[index].head<3>());
                    Eigen::Vector3d tempVec = screwAxisList_[index].segment<3>(3);
                    //Input JS should start from 0
                    transformList_[index] = inv(comList_[index]) * comList_[index + 1] * ExpTransA(tempSo3,tempVec, inputJS_[index][0]);

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