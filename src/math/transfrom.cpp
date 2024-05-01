#include "transform.hpp"

Transform::Transform() {
    clear();
}

Transform::~Transform() {}


void Transform::clear() {
    t.setIdentity();
}

Transform& Transform::translate(double x, double y, double z) {
    Eigen::Matrix4d translation;
    translation <<  1, 0, 0, x,
                    0, 1, 0, y,
                    0, 0, 1, z,
                    0, 0, 0, 1;
    t = t * translation;
    return *this;
}

Transform& Transform::translate(const Eigen::Vector3d& p){
    Eigen::Matrix4d translation;
    translation <<  1, 0, 0, p.x(),
                    0, 1, 0, p.y(),
                    0, 0, 1, p.z(),
                    0, 0, 0, 1;
    t = t * translation;
    return *this;
}


Transform& Transform::translateX(double x) {
    t(0,3) += t(0,0) * x;
    t(1,3) += t(1,0) * x;
    t(2,3) += t(2,0) * x;
    return *this;
}

Transform& Transform::translateY(double y) {
    t(0,3) += t(0,1) * y;
    t(1,3) += t(1,1) * y;
    t(2,3) += t(2,1) * y;
    return *this;
}

Transform& Transform::translateZ(double z) {
    t(0,3) += t(0,2) * z;
    t(1,3) += t(1,2) * z;
    t(2,3) += t(2,2) * z;
    return *this;
}


Transform& Transform::rotateX(double angle) {
    double c = cos(angle);
    double s = sin(angle);
    Eigen::Matrix4d rotation;
    rotation << 1, 0,  0, 0,
                0, c, -s, 0,
                0, s,  c, 0,
                0, 0,  0, 1;
    t = t * rotation;
    return *this;
}

Transform& Transform::rotateY(double angle) {
    double c = cos(angle);
    double s = sin(angle);
    Eigen::Matrix4d rotation;
    rotation << c,  0, s, 0,
                0,  1, 0, 0,
                -s, 0, c, 0,
                0,  0, 0, 1;
    t = t * rotation;
    return *this;
}

Transform& Transform::rotateZ(double angle) {
    double c = cos(angle);
    double s = sin(angle);
    Eigen::Matrix4d rotation;
    rotation << c, -s, 0, 0,
                s,  c, 0, 0,
                0,  0, 1, 0,
                0,  0, 0, 1;
    t = t * rotation;
    return *this;
}

Transform& Transform::rotateDotX(double a) {
    double ca = cos(a);
    double sa = sin(a);
    Eigen::Matrix4d rotDotX;
    rotDotX <<  1, 0,   0,   0,
                0, -sa, -ca, 0,
                0, ca,  -sa, 0,
                0, 0,   0,   1;
    t *= rotDotX;  // Apply the rotation derivative matrix
    return *this;
}

Transform& Transform::rotateDotY(double a) {
    double ca = cos(a);
    double sa = sin(a);
    Eigen::Matrix4d rotDotY;
    rotDotY << -sa, 0, ca,  0,
                0,  1, 0,   0,
                -ca, 0, -sa, 0,
                0,  0, 0,   1;
    t *= rotDotY;  // Apply the rotation derivative matrix
    return *this;
}

Transform& Transform::rotateDotZ(double a) {
    double ca = cos(a);
    double sa = sin(a);
    Eigen::Matrix4d rotDotZ;
    rotDotZ << -sa, -ca, 0, 0,
                ca, -sa, 0, 0,
                0,   0,  1, 0,
                0,   0,  0, 1;
    t *= rotDotZ;  // Apply the rotation derivative matrix
    return *this;
}


//d(rotateX(-q))/dt =  - d/dt(rotateX(-q))
Transform& Transform::rotateDotXNeg(double a) {
    double ca = cos(a);
    double sa = sin(a);
    Eigen::Matrix4d rotDotXNeg;
    rotDotXNeg << 1, 0,   0,   0,
            0, sa,  ca, 0,  // Note the inverted signs for sine components
            0, -ca, sa, 0,
            0, 0,   0,   1;
    t *= rotDotXNeg;  // Apply the negative rotation derivative matrix
    return *this;
}

Transform& Transform::rotateDotYNeg(double a) {
    double ca = cos(a);
    double sa = sin(a);
    Eigen::Matrix4d rotDotYNeg;
    rotDotYNeg <<   sa, 0, -ca, 0,
                    0,  1,  0,  0,
                    ca, 0,  sa, 0,
                    0,  0,  0,  1;
    t *= rotDotYNeg;  // Apply the negative rotation derivative matrix
    return *this;
}

Transform& Transform::rotateDotZNeg(double a) {
    double ca = cos(a);
    double sa = sin(a);
    Eigen::Matrix4d rotDotZNeg;
    rotDotZNeg <<   sa,  ca, 0, 0,
                    -ca, sa, 0, 0,
                    0,   0,  1, 0,
                    0,   0,  0, 1;
    t *= rotDotZNeg;  // Apply the negative rotation derivative matrix
    return *this;
}

void Transform::trcopy(Transform &out) {
    // Directly copy using Eigen's assignment operator
    out.t = t;
}

Transform& Transform::mDH(double alpha, double a, double theta, double d) {
    /*
    Transform t1;
    double ca = cos(alpha);
    double sa = sin(alpha);
    double ct = cos(theta);
    double st = sin(theta);
    t1(0,0) = ct; t1(0,1) = -st; t1(0,2) = 0; t1(0,3) = a;
    t1(1,0) = st*ca; t1(1,1) = ct*ca; t1(1,2) = -sa; t1(1,3) = -sa*d;
    t1(2,0) = st*sa; t1(2,1) = ct*sa; t1(2,2) = ca; t1(2,3) = ca*d;
    */

    this->translateX(a).rotateX(alpha).translateZ(d).rotateZ(theta);
    return *this;
}

void Transform::apply(Eigen::Vector3d& x) const{
    for (int i = 0; i < 3; i++) x[i] = t(i,3);
}


double Transform::getX() {return t(0,3);}
double Transform::getY() {return t(1,3);}
double Transform::getZ() {return t(2,3);}


double Transform::operator() (int i, int j) const {
    return t(i,j);
}



Transform operator* (const Transform &lhs, const Transform &rhs) {
    Transform result;
    result.t = lhs.t * rhs.t;
    return result;
}

Transform inv (const Transform &t1) {
    Transform result;
    result.t = t1.t.inverse();
    return result;
}


Transform transform6D(const Eigen::VectorXd p) {
    Transform result;
    // Extract parameters for clarity
    double px = p[0], py = p[1], pz = p[2];
    double rx = p[3], ry = p[4], rz = p[5];

    // Compute cosine and sine of angles
    double cx = cos(rx), sx = sin(rx);
    double cy = cos(ry), sy = sin(ry);
    double cz = cos(rz), sz = sin(rz);

    // Build rotation matrix using ZYX order
    Eigen::Matrix4d m;
    m(0,0) = cy * cz;
    m(0,1) = sx * sy * cz - cx * sz;
    m(0,2) = cx * sy * cz + sx * sz;
    m(0,3) = px;

    m(1,0) = cy * sz;
    m(1,1) = sx * sy * sz + cx * cz;
    m(1,2) = cx * sy * sz - sx * cz;
    m(1,3) = py;

    m(2,0) = -sy;
    m(2,1) = sx * cy;
    m(2,2) = cx * cy;
    m(2,3) = pz;

    m(3,0) = 0;
    m(3,1) = 0;
    m(3,2) = 0;
    m(3,3) = 1;

    result.t = m;  // Set the transformation matrix to the computed matrix
    return result;
}

Transform transformQuatP(const Eigen::VectorXd& q) {
    Transform result;
    // Calculate the rotation matrix components from the quaternion
    result.t(0,0) = 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3];
    result.t(0,1) = 2 * q[1] * q[2] - 2 * q[3] * q[0];
    result.t(0,2) = 2 * q[1] * q[3] + 2 * q[2] * q[0];
    result.t(0,3) = q[4];

    result.t(1,0) = 2 * q[1] * q[2] + 2 * q[3] * q[0];
    result.t(1,1) = 1 - 2 * q[1] * q[1] - 2 * q[3] * q[3];
    result.t(1,2) = 2 * q[2] * q[3] - 2 * q[1] * q[0];
    result.t(1,3) = q[5];

    result.t(2,0) = 2 * q[1] * q[3] - 2 * q[2] * q[0];
    result.t(2,1) = 2 * q[2] * q[3] + 2 * q[1] * q[0];
    result.t(2,2) = 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2];
    result.t(2,3) = q[6];

    result.t(3,0) = result.t(3,1) = result.t(3,2) = 0;
    result.t(3,3) = 1;  // Ensure the bottom row is correct for homogeneous coordinates
    return result;
}


Eigen::VectorXd position6D(const Transform &t1) {
    Eigen::VectorXd p(6);
    p[0] = t1(0,3);
    p[1] = t1(1,3);
    p[2] = t1(2,3);

    // Calculate Euler angles from the rotation matrix
    p[3] = atan2(t1(2,1), t1(2,2)); // Roll
    p[4] = asin(-t1(2,0)); // Pitch
    p[5] = atan2(t1(1,0), t1(0,0)); // Yaw

    // Handle gimbal lock scenario
    if (fabs(cos(p[4])) < 1E-6) {
        // Adjust calculations when pitch is near +/-90 degrees
        p[3] = atan2(-t1(1,2), t1(1,1));
        p[4] = asin(-t1(2,0));
        p[5] = 0;  // Yaw is undefined in this case
    }
    return p;
}


//not tested yet
Eigen::VectorXd to_quatp(const Transform &t) {
    Eigen::VectorXd q(7);
    double tr = t(0,0) + t(1,1) + t(2,2);

    if (tr > 0) {
        double s = sqrt(tr + 1.0) * 2;  // S is the sum of trace plus one, times two
        q[0] = 0.25 * s;
        q[1] = (t(2,1) - t(1,2)) / s;
        q[2] = (t(0,2) - t(2,0)) / s;
        q[3] = (t(1,0) - t(0,1)) / s;
    } else if (t(0,0) > t(1,1) && t(0,0) > t(2,2)) {
        double s = sqrt(1.0 + t(0,0) - t(1,1) - t(2,2)) * 2;
        q[0] = (t(2,1) - t(1,2)) / s;
        q[1] = 0.25 * s;
        q[2] = (t(0,1) + t(1,0)) / s;
        q[3] = (t(0,2) + t(2,0)) / s;
    } else if (t(1,1) > t(2,2)) {
        double s = sqrt(1.0 + t(1,1) - t(0,0) - t(2,2)) * 2;
        q[0] = (t(0,2) - t(2,0)) / s;
        q[1] = (t(0,1) + t(1,0)) / s;
        q[2] = 0.25 * s;
        q[3] = (t(1,2) + t(2,1)) / s;
    } else {
        double s = sqrt(1.0 + t(2,2) - t(0,0) - t(1,1)) * 2;
        q[0] = (t(1,0) - t(0,1)) / s;
        q[1] = (t(0,2) + t(2,0)) / s;
        q[2] = (t(1,2) + t(2,1)) / s;
        q[3] = 0.25 * s;
    }

    // Set the translation components
    q[4] = t(0,3);
    q[5] = t(1,3);
    q[6] = t(2,3);

    return q;
}

void getAngularVelocityTensor(const Transform &adot, const Transform &ainv,Eigen::MatrixXd angularVelosityTensor ){
    Transform result = adot*ainv;
    angularVelosityTensor = result.t;
}



void printTransform(Transform tr) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%.4g ", tr(i,j));
        }
        printf("\n");
    }
    printf("\n");
}

void printVector(Eigen::VectorXd v) {
    for (int i = 0; i < (int) v.size(); i++) {
        printf("%.4g\n", v[i]);
    }
    printf("\n");
}