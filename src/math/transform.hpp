//
// Created by gj on 4/25/24.
//

#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <array>
#include <math.h>

class Transform {

public:
    Transform();;
    void clear();

    Transform& translate(double x=0, double y=0, double z=0);
    Transform& translate(const Eigen::Vector3d& p);
    Transform& translateX(double x);
    Transform& translateY(double y);
    Transform& translateZ(double z);
    Transform& rotateX(double angle);
    Transform& rotateY(double angle);
    Transform& rotateZ(double angle);
    Transform& rotateDotX(double a);
    Transform& rotateDotY(double a);
    Transform& rotateDotZ(double a);
    Transform& rotateDotXNeg(double a);
    Transform& rotateDotYNeg(double a);
    Transform& rotateDotZNeg(double a);
    double getX();
    double getY();
    double getZ();

    void trcopy (Transform &out);

    void apply(Eigen::Vector3d& x) const;

    void getXYZ(Eigen::Vector3d& ret) const { ret = t.block<3,1>(0,3); }

    double operator() (int i, int j) const ;

    Transform& mDH(double alpha, double a, double theta, double d);

    Eigen::Matrix4d t;
};

Transform operator* (const Transform &lhs, const Transform &rhs);
Transform inv (const Transform &t);
Transform transform6D(const Eigen::VectorXd p);
Transform transformQuatP(const Eigen::VectorXd q);
Eigen::VectorXd position6D(const Transform &t);
Eigen::VectorXd to_quatp(const Transform &t);

void getAngularVelocityTensor(const Transform &adot, const Transform &ainv, Eigen::MatrixXd angularVelocityTensor);


void printTransform(Transform &t);
void printVector(Eigen::VectorXd &t);

#endif //TRANSFORM_HPP
