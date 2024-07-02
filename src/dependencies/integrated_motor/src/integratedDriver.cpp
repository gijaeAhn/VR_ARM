//
// Created by sj on 24. 7. 1.
//


#include "../include/integrated_driver.hpp"

namespace driver{

    integratedDriver::integratedDriver() {
        zmq::context_t context(1);
        robot_torque_subscriber_ = std::make_shared<zmq::socket_t>(context, ZMQ_SUB);
        rmd_angle_publisher_ = std::make_unique<zmq::socket_t>(context,ZMQ_PUB);
        dyna_angle_publisher_ = std::make_unique<zmq::socket_t>(context,ZMQ_PUB);
        std::make_

        std::stringstream rts;
        std::stringstream rap;
        std::stringstream dap;
        std::string host = "localhost";
        rts << "tcp://" << host << ":" << ROBOT_MOTOR_ADDR;
        rap << "tcp://*:" << RMD_ANGLE_ADDR;
        dap << "tcp://*"  << DYNAMIXEL_ANGLE_ADDR;
        robot_torque_subscriber_.connect(rts.str());
        rmd_angle_publisher.bind(rap.str());
        dyna_angle_publisher_.bind(dap.str());
        zmq::sockopt::array_option<ZMQ_SUBSCRIBE,1> sockopt;
        robot_torque_subscriber.set(sockopt,"");

        //ARRAY INIT
        rmdCurrentShaft.fill(0);
        rmdPreviousShaft.fill(0);
        rmdShaftChange.fill(0);

        angBuffer.fill(0);







    }


    integratedDriver::~integratedDriver() {

    }

    void integratedDriver::run() {

        // 2 functions is running on threads
        // 1. dynamixel driver
        // 2. rmd driver
        if(dynaThread_.joinable() && rmdThread_.joinable()){
            running_ = true;
            dynaThread_.join();
            rmdThread_.join();
        }else{
            std::vector<std::string> debug_stream;
            if(!dynaThread_.joinable()){
                debug_stream.push_back("Dynamixel Thread");
            }
            if(!rmdThread_.joinable()){
                debug_stream.push_back("RMD Thread");
            }
            if(!debug_stream.empty()){
                std::cout << debug::red_expression << "Thread Join Error" << debug::reset_expression;
                bool first = true
                for(const auto& value : debug_stream){
                    if(!fisrt){
                        std::cout << ", ";
                    }
                    std::cout << value;
                    frist = false;
                }
                std::cout << std::flush;
            }
        }
    }

    void integratedDriver::terminate() {

    }

    void integratedDriver::torqueSubFunc() {
        zmq::message_t torque_sub_message(sizeof(SYSTEM_PRECISION_TYPE) * ROBOT_MOTOR_SIZE);
        auto torque_sub_messgae_ptr = reinterpret_cast<SYSTEM_PRECISION_TYPE*>(torque_sub_message.data());
        while(running_){
            auto threadFuncStartTime std::chrono::steady_clock::now();

            std::unique_lock<std::mutex> lock(torqueMutex_);
            {
                //Subscribe Torque and Set it
                zmq::recv_result_t result = robot_torque_subscriber_.recv(torque_sub_message,zmq::recv_flags::none);
                if(!result){
                    std::cout << debug::red_expression << "Driver RECV Failure : Entire Torque" << std::flush;
                }else{
                    std::copy(torque_sub_messgae_ptr,torque_sub_messgae_ptr + ROBOT_MOTOR_SIZE, robotTorque_);
                    std::cout << debug::green_expression << "RECV MOTOR TORQUE :"
                    for(size_t index = 0; index < ROBOT_MOTOR_SIZE ; index++){



                    }
                }
            }

        }

    }

    void integratedDriver::dynaRunningFunc() {

        zmq::message_t dyna_angle_message(sizeof(SYSTEM_PRECISION_TYPE) * DYNAMIXEL_MOTOR_SIZE);
        auto dyna_angle_messgae_ptr = reinterpret_cast<SYSTEM_PRECISION_TYPE*>(dyna_angle_message.data());

        //INIT STAGE


        //INIT STAGE END

        while(running_){
        }

    }


    void integratedDriver::rmdRunningFunc() {

        zmq::message_t rmd_angle_message(sizeof(SYSTEM_PRECISION_TYPE) * RMD_MOTOR_SIZE);
        auto rmd_angle_message_ptr = reinterpret_cast<SYSTEM_PRECISION_TYPE*>(rmd_angle_message.data());

        //INIT STAGE

        //INIT STAGE END

        while(running_){

        }
    }




}

