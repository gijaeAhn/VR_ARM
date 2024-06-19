//
// Created by gj on 24. 3. 5.
//

#ifndef VR_ARM_ROBOT_BODY_HPP
#define VR_ARM_ROBOT_BODY_HPP

#include <memory>
#include <array>
#include <Eigen/Dense>
#include <thread>
#include <mutex>

#include "fsm.hpp"
#include "math/rigidBody.hpp"
#include "math/armDynamcisSolver.hpp"
#include "math/armKinematicsSolver.hpp"

#include "utilities/include/debug.hpp"
#include "utilities/include/param.hpp"
#include "utilities/include/address.hpp"

#ifdef __APPLE__
#include "/opt/homebrew/Caskroom/miniforge/base/envs/ros_env/include/zmq.hpp"
#endif
#ifdef __linux__
#include <zmq.hpp>
#endif



namespace robot {
    class Robot {
    public:
        Robot(std::string name,
              std::vector<param::LinkParam> inputLinkParams,
              std::vector<param::DHParam> inputDHParams,
              std::vector<Eigen::VectorXd> inputA);

        ~Robot() = default;

        void run();

    private:

        //* State Variables
        //  States Should change according to VR Button Combinations
        fsm::FiniteStateMachine fsm_;

        //* Sharing Variables
        std::shared_ptr<Transform> EETransformPtr_;
        std::shared_ptr<std::vector<param::JointState>> jointStatePtr_;
        std::shared_ptr<Eigen::VectorXd> torquePtr_;

        Transform EETransform_;
        std::vector<param::JointState> jointState_;
        std::vector<param::JointState> estimatedJointState_;
        Eigen::VectorXf torque_;


        //For threads
        void ikSolverThreadFunc();
        void dynamicsSolverThreadFunc();
        void zmqThreadFunc();

        std::atomic<bool> running_;
        std::thread ikSolverThread_;
        std::thread dynamicsSolverThread_;
        std::thread zmqThread_;

        std::mutex eeTransformMutex_;
        std::mutex torqueMutex_;
        std::mutex jointStateMutex_;



        //1. Get Transformation function
        //2.
        math::armKinematics::armKinematicsSolver ikSolver_;
        math::armDynamics::armDynamicsSolver dynamicsSolver_;

        //ZMQ
        zmq::socket_t EETransSubSocket_;
        zmq::socket_t torquePubSocket_;
        zmq::socket_t estimateStateSubSocket_;

        // Debug Functions

        //Internal Functions
        void getJointStates();
        Transform deserializeTransform(const zmq::message_t& message);
        std::vector<param::JointState> deserializationJS(const zmq::message_t message);

        std::string name_;
        uint8_t dof_;



    };
}
#endif //ROBOT_BODY_HPP
