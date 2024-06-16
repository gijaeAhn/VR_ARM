//
// Created by gj on 24. 4. 2.
//

#ifndef VR_ARM_PARSER_HPP
#define VR_ARM_PARSER_HPP
#pragma once

#ifdef __APPLE__
#endif

#ifdef __linux__
#endif

#include <iostream>
#include <filesystem>
#include <fstream>
#include <optional>

#include "param.hpp"

namespace utilities {

    namespace parser {

        enum PARSE_RESULT {
            PARSE_SUCCEED,
            PARSE_FAILURE,
            PARSE_ERROR
        };

        enum GET_RESULT{
            GET_SUCCEED,
            GET_FAILURE
        };

        enum PATH_VALIDATION_RESULT {
            PATH_VALID,
            PATH_INVALID
        };

        enum PERMISSION_CHECK_RESULT{
            ADMIN_PERMISSION,
            RW_PERMISSION,
            NO_PERMISSION
        };



        template<typename Data>
        class Parser {

        public:
            Parser() = default;
            virtual ~Parser() = default;

            struct outputDataBundle{
                Data data;
                GET_RESULT getResult;
            };

            virtual PERMISSION_CHECK_RESULT permissionCheck();
            virtual PATH_VALIDATION_RESULT getConfigPath(const std::string& configPath);
            virtual PATH_VALIDATION_RESULT getDataSavePath(const std::string& dataSavePath);
            virtual PARSE_RESULT parse();
            virtual std::optional<outputDataBundle> getData() = 0;


        protected:
            Data data_;
            std::string configPath_;
            std::string dataSavingPath_;

            // Inner Functions
            void saveData(Data inputData);
        };

        template<typename Data>
        std::optional<typename Parser<Data>::outputDataBundle> Parser<Data>::getData() {
            return Parser<Data>::outputDataBundle();
        }

        template<typename Data>
        void Parser<Data>::saveData(Data inputData) {
            // Implement the save data logic here
        }

    }

}

#endif // VR_ARM_PARSER_HPP