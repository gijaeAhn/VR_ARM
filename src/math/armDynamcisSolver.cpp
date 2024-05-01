//
// Created by gj on 4/29/24.
//
#include "armDynamcisSolver.hpp"

namespace math{
    namespace armDynamics{
        armDynamicsSolver::armDynamicsSolver() {
            dof_ = 6;
            inputJS_.resize(3);
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

        void armDynamicsSolver::updateTransform() {
            //t


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