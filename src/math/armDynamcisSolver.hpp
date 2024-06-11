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


namespace math {
    namespace armDynamics {

        using namespace std;
        using namespace Eigen;

        // JointState => thetalist, thetaDotlist, thetaDoubleDotlist
        // torqueList
        using torqueList = VectorXd;

        class armDynamicsSolver : public Solver<vector<JointState>, torqueList, vector<LinkParam>> {

        public:

            armDynamicsSolver();
            ~armDynamicsSolver();

            void solve() override;
            void getDhParam(const vector<DHParam>& inputDhList);
            void getLinkParam(const vector<LinkParam>& inputLinkParam);
            void getJointState(const vector<JointState>& inputJointState);

        private:

            //* Variables
            vector<JointState> inputJointState_;
            vector<LinkParam> linkParams_;
            vector<DHParam> dhParams_;
            vector<VectorXd> A_;
            vector<double> solutionTorque_;


            uint8_t dof_= 6;
            double shoulderHeight;
            double upperArmLength;
            double lowerArmLength;
            double wrist1x;
            double wrist1z;
            double wrist2x;
            double wrist2z;
            //*//

            //* Functions
            void calculateTorques();
        };
    }
}

#endif //VR_ARM_ARMDYNAMICSSOLVER_HPP