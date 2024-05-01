//
// Created by gj on 4/29/24.
//

#ifndef VR_ARM_ARMDYNAMCISSOLVER_HPP
#define VR_ARM_ARMDYNAMCISSOLVER_HPP

#include "Solver.hpp"
#include "transform.hpp"
#include "eigen3/Eigen/Dense"


namespace math {
    namespace armDynamics{

        using jointState = Eigen::VectorXd;
        using torqueList = Eigen::VectorXd;
        using spatialInertialMatrix = Eigen::MatrixXd;

        class armDynamicsSolver : public Solver<jointState, Eigen::VectorXd> {

        public:

            armDynamicsSolver();
            ~armDynamicsSolver();

            void solve();
            void apply(torqueList& cp);


            void getInput(jointState&& inputJS);
            void getError(Eigen::VectorXd error);
            void getScrewAxis(Eigen::VectorXd screwAxis, uint8_t index);
            void getScrewAxis(std::vector<Eigen::VectorXd> sal);
            void getSpatialInertialMatrix(Eigen::MatrixXd sim, uint8_t index);
            void getSpatialInertialMatrix(std::vector<Eigen::MatrixXd> matrixlist);

            void updateTransform();

            void calculateMassMatrix();
            void calculateLMatrix();
            void calculateCoriolis();
            void calculateGravity();



        private:
            uint8_t dof_;
            jointState inputJS_;
            Eigen::VectorXd inputError_;
            torqueList solution_;
            Eigen::MatrixXd massMatrix_;


            Eigen::MatrixXd lMatrix_;
            Eigen::VectorXd coriolis_;
            Eigen::VectorXd gravityComp_;

            std::vector<Eigen::MatrixXd> spatialInertialMatrixList_;
            spatialInertialMatrix spatialInertialMatrix_;

            std::vector<Eigen::VectorXd> screwAxisList_;
            Eigen::MatrixXd screwAxisMatrix_;

            std::vector<Eigen::Matrix4d> comList_;
            std::vector<Transform> transformList_;



        };
}
}


#endif //VR_ARM_ARMDYNAMCISSOLVER_HPP