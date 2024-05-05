//
// Created by gj on 5/1/24.
//

#ifndef VR_ARM_RIGIDBODY_HPP
#define VR_ARM_RIGIDBODY_HPP
#include "transform.hpp"
#include <eigen3/Eigen/Dense>

#define PI 3.14159265359

namespace math {



    bool NearZero(const double val) {
        return (std::abs(val) < .000001);
    }

    Eigen::MatrixXd Normalize(Eigen::MatrixXd V) {
        V.normalize();
        return V;
    }

    Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg) {
        Eigen::Matrix3d m_ret;
        m_ret << 0, -omg(2), omg(1),
                omg(2), 0, -omg(0),
                -omg(1), omg(0), 0;
        return m_ret;
    }

    Eigen::Vector3d so3ToVec(const Eigen::MatrixXd& so3mat) {
        Eigen::Vector3d v_ret;
        v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
        return v_ret;
    }

    Eigen::MatrixXd ad(Eigen::VectorXd V) {
        Eigen::Matrix3d omgmat = VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

        Eigen::MatrixXd result(6, 6);
        result.topLeftCorner<3, 3>() = omgmat;
        result.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero(3, 3);
        result.bottomLeftCorner<3, 3>() = VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
        result.bottomRightCorner<3, 3>() = omgmat;
        return result;
    }

    Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3) {
        Eigen::Vector4d v_ret;
        v_ret << Normalize(expc3), expc3.norm();
        return v_ret;
    }

    Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
        Eigen::MatrixXd m_ret(4, 4);
        m_ret << R, p,
                0, 0, 0, 1;
        return m_ret;
    }

    std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T) {
        std::vector<Eigen::MatrixXd> Rp_ret;
        Eigen::Matrix3d R_ret;
        // Get top left 3x3 corner
        R_ret = T.block<3, 3>(0, 0);

        Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

        Rp_ret.push_back(R_ret);
        Rp_ret.push_back(p_ret);

        return Rp_ret;
    }

    Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T) {

        std::vector<Eigen::MatrixXd> R = TransToRp(T);
        Eigen::MatrixXd ad_ret(6, 6);
        ad_ret = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
        ad_ret << R[0], zeroes,
                VecToso3(R[1]) * R[0], R[0];
        return ad_ret;
    }

    Eigen::Matrix3d RodriguesFormula(const Eigen::Matrix3d so3, double theta){
        Eigen::Matrix3d result;
        Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
        result = eye + std::sin(theta) * so3  + (1-std::cos(theta)) * so3*so3;
        return result;
    }

    Eigen::Matrix3d GFormula(const Eigen::Matrix3d so3, double theta){
        Eigen::Matrix3d result;
        Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
        result = eye * theta + (1 - std::cos(theta)) * so3 + (theta - std::sin(theta)) * so3 *so3;
        return result;
    }

    Transform ExpTransA(const Eigen::Matrix3d so3,Eigen::Vector3d v, double theta){
        Transform result;
        result.clear();
        Eigen::Matrix3d R = RodriguesFormula(so3, theta);
        Eigen::Matrix3d G = GFormula(so3, theta);
        // Test needed
        Eigen::Vector3d gv = G * v;
        Eigen::Vector4d temp = {0,0,0,1};

        result.t.topLeftCorner<3,3>() = R;
        result.t.topRightCorner<3,1>() = gv;
        result.t.bottomLeftCorner<4,1>() = temp;

        return result;
    }




}





#endif //VR_ARM_RIGIDBODY_HPP
