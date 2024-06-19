//
// Created by gj on 24. 3. 5.
//

#ifndef VR_ARM_ROBOT_BUILDER_HPP
#define VR_ARM_ROBOT_BUILDER_HPP

#include "robot_body.hpp"

namespace robot {

        class RobotBuilder {
            public:
            RobotBuilder() {}

            void addName(std::string&& name) {
                name_ = std::move(name);
            }

            void addA(std::vector<Eigen::VectorXd>&& A) {
                A_ = std::move(A);
            }

            void addDHParam(std::vector<param::DHParam>&& dhParam) {
                dhParam_ = std::move(dhParam);
            }

            void addLinkParams(std::vector<param::LinkParam>&& linkParam) {
                linkParam_ = std::move(linkParam);
            }

            Robot build() {
                return Robot(name_, linkParam_, dhParam_, A_);
            }

            private:
            std::string name_;
            std::vector<Eigen::VectorXd> A_;
            std::vector<param::DHParam> dhParam_;
            std::vector<param::LinkParam> linkParam_;
            // Assuming Robot is constructible with these parameters
        };

}
#endif //VR_ARM_ROBOT_BUILDER_HPP
