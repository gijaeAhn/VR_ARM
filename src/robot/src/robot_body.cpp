//
// Created by GJ on 6/13/24.
//

#include "include/robot_body.hpp"



namespace  robot{

    Robot::Robot(std::vector<param::LinkParam> inputLinkParams,
                 std::vector<param::DHParam> inputDHParams,
                 std::vector<Eigen::VectorXd> inputA)
    :EETransformPtr_(std::make_shared<Transform>()),
     jointStatePtr_(std::make_shared<std::vector<param::JointState>>()),
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
        std::stringstream estimateStateSub;
        std::stringstream torquePub;
        std::string host = "localhost";
        eeTransSub       << "tcp://" << host << ":" << EETRANS_SUB_ADDR;
        estimateStateSub << "tcp://" << host << ":" << ESTIMATE_STATE_ADDR;
        torquePub        << "tcp://*" << TORQUE_PUB_ADDR;
        EETransSubSocket_.connect(eeTransSub.str());
        EETransSubSocket_.set(sockOptSub, "");
        estimateStateSubSocket_.connect(estimateStateSub.str());
        estimateStateSubSocket_.set(sockOptSub,"");
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
                std::cout << "Ik Thread Actual Cycle is slower than set Cycle : Robot" << std::flush;
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
                std::cout << "Dynamics Thread Actual Cycle is slower than set Cycle : Robot" << std::flush;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(ZMQ_THREAD_CYCLE) - threadActualDuration);
            }
        }
    }

    void Robot::zmqThreadFunc() {
        while(running_){

            auto threadFuncStart = std::chrono::steady_clock::now();

            //Recveing Part


            // add Checking code
            // 1. Use Quaternion characteristic
            // 2. Lock the EE Transform with mutex

            std::unique_lock<std::mutex> lock1(eeTransformMutex_);
            {
                zmq::message_t eeTransSub_message(sizeof(float) * 7);
                zmq::recv_result_t recv_result = EETransSubSocket_.recv(eeTransSub_message, zmq::recv_flags::none);
                if (!(recv_result.has_value() && (recv_result.value() > 0))) {
                    std::cerr << ("Failed to recv TF : Robot");
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
                    std::cerr << ("Failed to send Torque : Robot");
                } else {
                    // Succeed
                }
            }
            lock2.unlock();

            std::lock_guard<std::mutex> lock3(jointStateMutex_);
            {
                size_t js_message_size = sizeof(float) * dof_ * 3;
                zmq::message_t estimateStateSub_message(js_message_size);
                zmq::recv_result_t  recv_result = estimateStateSubSocket_.recv(estimateStateSub_message, zmq::recv_flags::none);
                if (!(recv_result.has_value() && (recv_result.value() > 0))) {
                    std::cerr << ("Failed to recv Estimated State : Robot");
                } else {
                    estimatedJointState_ =  deserializeJS(estimateStateSub_message);
                }
            }

            zmq::message_t robotActualState(sizeof(float)*dof_);
            recv_result =


            auto threadFuncEnd = std::chrono::steady_clock::now();
            auto threadActualDuration = std::chrono::duration_cast<std::chrono::milliseconds>(threadFuncEnd - threadFuncStart);

            if (threadActualDuration.count() > ZMQ_THREAD_CYCLE) {
                std::cout << "ZMQ Thread Actual Cycle is slower than set Cycle : Robot" << std::flush;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(ZMQ_THREAD_CYCLE) - threadActualDuration);
            }
        }

    }

    Transform Robot::deserializeTransform(const zmq::message_t& message){
        if (message.size() != sizeof(SYSTEM_PRECISION_TYPE) * 7) {
            throw std::runtime_error("Invalid message size for Transform deserialization : Robot");
        }

        const float* buffer = reinterpret_cast<const SYSTEM_PRECISION_TYPE*>(message.data());
        SYSTEM_PRECISION_TYPE rotationY = buffer[1];
        SYSTEM_PRECISION_TYPE rotationZ = buffer[2];
        SYSTEM_PRECISION_TYPE rotationW = buffer[3];
        SYSTEM_PRECISION_TYPE rotationX = buffer[0];
        SYSTEM_PRECISION_TYPE translationX = buffer[4];
        SYSTEM_PRECISION_TYPE translationY = buffer[5];
        SYSTEM_PRECISION_TYPE translationZ = buffer[6];

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

    std::vector<param::JointState> Robot::deserializationJS(const zmq::message_t& message) {
        const SYSTEM_PRECISION_TYPE* buffer = reinterpret_cast<const SYSTEM_PRECISION_TYPE*>(message.data());
        size_t expected_size = sizeof(SYSTEM_PRECISION_TYPE_PRECISION_TYPE) * dof_ * 3;

        // Check if the message size is as expected
        if (message.size() != expected_size) {
            throw std::runtime_error("Deserialize Joint State from Motor Failed. : Robot");
        }

        std::vector<param::JointState> temp_js(dof_); // Pre-size the vector to avoid reallocations

        for (size_t i = 0; i < dof_; i++) {
            size_t base_index = 3 * i;
            temp_js[i].positionAngle = buffer[base_index];
            temp_js[i].velocityAngle = buffer[base_index + 1];
            temp_js[i].accelerationAngle = buffer[base_index + 2];
        }

        return temp_js;
    }

}