//
// Created by GJ on 6/16/24.
//

#ifndef VR_ARM_PARSER_YAML_ARM_HPP
#define VR_ARM_PARSER_YAML_ARM_HPP

#include "param.hpp"
#include "parser.hpp"

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>


namespace utilities::parser {


    class YamlParser : public Parser<param::ArmConfigParam> {
    public:
        YamlParser() = default;

        virtual ~YamlParser() = default;

        virtual PARSE_RESULT parse() override;

        virtual PERMISSION_CHECK_RESULT permissionCheck() override;

        virtual PATH_VALIDATION_RESULT getConfigPath(const std::string &configPath) override;

        virtual std::optional<typename Parser<param::ArmConfigParam>::outputDataBundle> getData() override;

    };

    PERMISSION_CHECK_RESULT YamlParser::permissionCheck() {
        // Implement permission check logic
        return utilities::parser::RW_PERMISSION;
    }

    PATH_VALIDATION_RESULT YamlParser::getConfigPath(const std::string &configPath) {
        if (std::filesystem::exists(configPath)) {
            configPath_ = configPath;
            return PATH_VALID;
        } else {
            return PATH_INVALID;
        }
    }

    PARSE_RESULT YamlParser::parse() {
        try {
            YAML::Node config = YAML::LoadFile(configPath_);

            if (config["links"]) {
                for (const auto &linkNode: config["links"]) {
                    param::LinkParam link{};
                    auto linkName = linkNode.first.as<std::string>();
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
                    link.inertiaTensor(1, 0) = linkNode.second["inertial"]["Lxy"].as<double>(); // symmetric
                    link.inertiaTensor(1, 1) = linkNode.second["inertial"]["Lyy"].as<double>();
                    link.inertiaTensor(1, 2) = linkNode.second["inertial"]["Lyz"].as<double>();
                    link.inertiaTensor(2, 0) = linkNode.second["inertial"]["Lxz"].as<double>(); // symmetric
                    link.inertiaTensor(2, 1) = linkNode.second["inertial"]["Lyz"].as<double>(); // symmetric
                    link.inertiaTensor(2, 2) = linkNode.second["inertial"]["Lzz"].as<double>();

                    data_.links[linkName] = link;
                }
            }

            if (config["joints"]) {
                for (const auto &jointNode: config["joints"]) {
                    param::DHParam joint{};
                    auto jointName = jointNode.first.as<std::string>();
                    joint.alpha = jointNode.second["alpha"].as<double>();
                    joint.a = jointNode.second["a"].as<double>();
                    joint.d = jointNode.second["d"].as<double>();
                    joint.theta = jointNode.second["theta"].as<double>();

                    data_.joints[jointName] = joint;
                }
            }
        } catch (const std::exception &e) {
            std::cerr << "Failed to parse YAML file: " << e.what() << std::endl;
        }
    }

    std::optional<typename utilities::parser::Parser<param::ArmConfigParam>::outputDataBundle>
    YamlParser::getData() {
        if (data_.links.empty() && data_.joints.empty()) {
            return std::nullopt;
        } else {
            return typename utilities::parser::Parser<param::ArmConfigParam>::outputDataBundle{data_,
                                                                                               parser::GET_RESULT::GET_SUCCEED};
        }
    }
}






#endif //VR_ARM_PARSER_YAML_ARM_HPP
