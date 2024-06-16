//
// Created by GJ on 6/16/24.
//

#ifndef VR_ARM_PARSER_YAML_ARM_HPP
#define VR_ARM_PARSER_YAML_ARM_HPP

#include "param.hpp"
#include "parser.hpp"

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>


namespace utilities{
    namespace parser{



        class YamlParser : public utilities::parser::Parser<param::ArmConfigParam> {
        public:
            YamlParser() = default;
            virtual ~YamlParser() = default;

            virtual utilities::parser::PARSE_RESULT parse() override;
            virtual utilities::parser::WRITE_RESULT writeData() override;
        };

        PARSE_RESULT YamlParser::parse() {
            try {
                YAML::Node config = YAML::LoadFile(configPath_);

                if (config["links"]) {
                    for (const auto& linkNode : config["links"]) {
                        param::LinkParam link;
                        std::string linkName = linkNode.first.as<std::string>();
                        link.mass = linkNode.second["mass"].as<double>();
                        link.centerOfMass = Eigen::Vector3d(
                                linkNode.second["com"]["x"].as<double>(),
                                linkNode.second["com"]["y"].as<double>(),
                                linkNode.second["com"]["z"].as<double>()
                        );
                        link.inertiaTensor = Eigen::Matrix3d::Zero();
                        link.inertiaTensor(0, 0) = linkNode.second["inertial"]["Lxx"].as<double>();
                        link.inertiaTensor(0, 1) = linkNode.second["inertial"]["Lxy"].as<double>();
                        link.inertiaTensor(0, 2) = linkNode.second["inertial"]["Lxz"].as<double>();
                        link.inertiaTensor(1, 1) = linkNode.second["inertial"]["Lyy"].as<double>();
                        link.inertiaTensor(1, 2) = linkNode.second["inertial"]["Lyz"].as<double>();
                        link.inertiaTensor(2, 2) = linkNode.second["inertial"]["Lzz"].as<double>();

                        // Fill the symmetric elements
                        link.inertiaTensor(1, 0) = link.inertiaTensor(0, 1);
                        link.inertiaTensor(2, 0) = link.inertiaTensor(0, 2);
                        link.inertiaTensor(2, 1) = link.inertiaTensor(1, 2);

                        data_.links[linkName] = link;
                    }
                }

                if (config["joints"]) {
                    for (const auto& jointNode : config["joints"]) {
                        param::DHParam joint;
                        std::string jointName = jointNode.first.as<std::string>();
                        joint.alpha = jointNode.second["alpha"].as<double>();
                        joint.a = jointNode.second["a"].as<double>();
                        joint.d = jointNode.second["d"].as<double>();
                        joint.theta = jointNode.second["theta"].as<double>();

                        data_.joints[jointName] = joint;
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "Failed to parse YAML file: " << e.what() << std::endl;
            }
        }



    }
}





#endif //VR_ARM_PARSER_YAML_ARM_HPP
