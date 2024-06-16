//
// Created by GJ on 6/13/24.
//

#include "include/robot_body.hpp"



namespace  robot{

    Robot::Robot(std::vector<math::LinkParam> inputLinkParams,
                 std::vector<math::DHParam> inputDHParams,
                 std::vector<Eigen::VectorXd> inputA)
    :EETransformPtr_(std::make_shared<Transform>()),
     jointStatePtr_(std::make_shared<std::vector<math::JointState>>()),
     torquePtr_(std::make_shared<Eigen::VectorXd>()),
     running_(false),
     ikSolver_(EETransformPtr_,jointStatePtr_,inputDHParams),
     dynamicsSolver_(inputLinkParams, inputDHParams, inputA, jointStatePtr_, torquePtr_){

        dof_ = 6;

        // Thread Setting
        ikSolverThread_       = std::thread(&Robot::ikSolverThreadFunc, this);
        dynamicsSolverThread_ = std::thread(&Robot::dynamicsSolverThreadFunc,this);
        zmqThread_            = std::thread(&Robot::zmqThread_, this);

        // ZMQ Setting
        zmq::context_t context(1);
        zmq::sockopt::array_option<ZMQ_SUBSCRIBE,1> sockOptSub;
        zmq::sockopt::array_option<ZMQ_PUB,1> sockOptPub;
        std::stringstream eeTransSub;
        std::stringstream eeTransPub;
        std::string host = "localhost";
        eeTransSub << "tcp://" << host << ":" << EETRANS_SUB_ADDR;
        eeTransPub << "tcp://*" << EETRANS_PUB_ADDR;
        EETransSubSocket_.connect(eeTransSub.str());
        EETransSubSocket_.set(sockOptSub, "");
        torquePubSocket_.bind(eeTransPub.str());
        torquePubSocket_.set(sockOptPub,"");

    }

    void Robot::run() {

        // There are 3 functions which should be running on threads
        // 1. IK Solver
        // 2. Dynamics Solver
        // 3. Motor Torque Message

        if(ikSolverThread_.joinable() && dynamicsSolverThread_.joinable() && zmqThread_.joinable()){
            running_ = true;
            ikSolverThread_.join();
            dynamicsSolverThread_.join();
            zmqThread_.join();
        }else{
            std::vector<std::string> debug_stream;
            if (!ikSolverThread_.joinable()) {
                debug_stream.push_back("ikSolver");
            }
            if (!dynamicsSolverThread_.joinable()) {
                debug_stream.push_back("dynamicsSolver");
            }
            if (!zmqThread_.joinable()) {
                debug_stream.push_back("zmq");
            }
            if(!debug_stream.empty()){
                std::cout << debug::red_expression << "Thread Join Error" << debug::reset_expression;
                for(uint8_t i =0; i<debug_stream.size();i++){
                    if(i != 0){
                        std::cout << ", ";
                    }
                    std::cout << debug_stream[i];
                }
                std::cout << std::flush;
            }
        }
    }

    void Robot::ikSolverThreadFunc() {
        while (running_) {
            auto threadFuncStart = std::chrono::steady_clock::now();

            std::unique_lock<std::mutex> lock(jointStateMutex_);
            {
                if (EETransformPtr_) {
                    ikSolver_.getInput();
                    ikSolver_.solve();
                    ikSolver_.setJointState();
                } else {
                    //implement asynchronous debuggin UI
                    continue;
                }
            }
            lock.unlock();

            auto threadFuncEnd = std::chrono::steady_clock::now();
            auto threadActualDuration = std::chrono::duration_cast<std::chrono::milliseconds>(threadFuncEnd - threadFuncStart);

            if (threadActualDuration.count() > ZMQ_THREAD_CYCLE) {
                std::cout << "Ik Thread Actual Cycle is slower than set Cycle" << std::flush;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(IK_THREAD_CYCLE) - threadActualDuration);
            }


        }
    }

    void Robot::dynamicsSolverThreadFunc() {
        while (running_) {

            auto threadFuncStart = std::chrono::steady_clock::now();

            std::unique_lock<std::mutex> lock(torqueMutex_);
            {
                if (jointStatePtr_) {
                    dynamicsSolver_.getJointState();
                    dynamicsSolver_.solve();
                    dynamicsSolver_.setTorques();
                }else{
                    //implement debugging UI
                    continue;
                }
            }
            lock.unlock();

            auto threadFuncEnd = std::chrono::steady_clock::now();
            auto threadActualDuration = std::chrono::duration_cast<std::chrono::milliseconds>(threadFuncEnd - threadFuncStart);

            if (threadActualDuration.count() > ZMQ_THREAD_CYCLE) {
                std::cout << "Dynamics Thread Actual Cycle is slower than set Cycle" << std::flush;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(ZMQ_THREAD_CYCLE) - threadActualDuration);
            }
        }
    }

    void Robot::zmqThreadFunc() {
        while(running_){

            auto threadFuncStart = std::chrono::steady_clock::now();

            //Recveing Part
            zmq::message_t eeTransSub_message(sizeof(float) * 7);
            zmq::recv_result_t recv_result = EETransSubSocket_.recv(eeTransSub_message, zmq::recv_flags::none);

            // add Checking code
            // 1. Use Quaternion characteristic
            // 2. Lock the EE Transform with mutex

            std::unique_lock<std::mutex> lock1(eeTransformMutex_);
            {
                if (!(recv_result.has_value() && (recv_result.value() > 0))) {
                    throw std::runtime_error("Failed to recv TF : Robot");
                } else {
                    EETransform_ = deserializeTransform(eeTransSub_message);
                }
            }
            lock1.unlock();

            // Publishing part

            zmq::message_t torquePub_message(sizeof(float)*dof_);
            auto tempMsgPtr = static_cast<float*>(torquePub_message.data());
            // Should check

            std::unique_lock<std::mutex> lock2(torqueMutex_);
            {
                std::copy(tempMsgPtr, tempMsgPtr + dof_, torque_.data());
                zmq::send_result_t send_result = torquePubSocket_.send(torquePub_message, zmq::send_flags::none);
                if (!(send_result.has_value() && (send_result.value() > 0))) {
                    throw std::runtime_error("Failed to send Torque : Robot");
                } else {
                    // Succeed
                }
            }
            lock2.unlock();

            auto threadFuncEnd = std::chrono::steady_clock::now();
            auto threadActualDuration = std::chrono::duration_cast<std::chrono::milliseconds>(threadFuncEnd - threadFuncStart);

            if (threadActualDuration.count() > ZMQ_THREAD_CYCLE) {
                std::cout << "ZMQ Thread Actual Cycle is slower than set Cycle" << std::flush;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(ZMQ_THREAD_CYCLE) - threadActualDuration);
            }
        }

    }

    Transform Robot::deserializeTransform(const zmq::message_t& message){
        if (message.size() != sizeof(double) * 7) {
            throw std::runtime_error("Invalid message size for Transform deserialization");
        }

        const double* buffer = reinterpret_cast<const double*>(message.data());
        double rotationX = buffer[0];
        double rotationY = buffer[1];
        double rotationZ = buffer[2];
        double rotationW = buffer[3];
        double translationX = buffer[4];
        double translationY = buffer[5];
        double translationZ = buffer[6];

        Transform transform;
        transform.t = Eigen::Matrix4d::Identity();

        // Set the translation part
        transform.t(0, 3) = translationX;
        transform.t(1, 3) = translationY;
        transform.t(2, 3) = translationZ;

        // Set the rotation part using a quaternion
        Eigen::Quaterniond quaternion(rotationW, rotationX, rotationY, rotationZ);
        Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();
        transform.t.block<3, 3>(0, 0) = rotationMatrix;

        return transform;
    }


}