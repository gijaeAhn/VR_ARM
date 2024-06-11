//
// Created by GJ on 5/30/24.
//

#ifndef VR_ARM_FSM_HPP
#define VR_ARM_FSM_HPP
#include <stdio.h>
#include <iostream>





namespace fsm {

    enum class State {
        IdleStage,
        InitStage,
        MockingStage,
        Emergency
    };

    enum class Event {
        SystemOn,
        InitDone,
        ActionStart,
        ActionEnd,
        Reset
    };

    class FiniteStateMachine {
    private:
        State currentState;

    public:
        FiniteStateMachine() : currentState(State::IdleStage) {}

        // Event handling method
        void handleEvent(Event event) {
            switch (currentState) {
                case State::IdleStage:
                    if (event == Event::InitDone) {
                        currentState = State::InitStage;
                        onInit();
                    }
                    break;
                case State::InitStage:
                    if (event == Event::ActionStart) {
                        currentState = State::MockingStage;
                        onMocking();
                    }
                    break;
                case State::MockingStage:
                    if (event == Event::ActionEnd) {
                        currentState = State::InitStage;
                        onInit();
                    }
                case State::Emergency:
                    if (event == Event::Reset) {
                        currentState = State::IdleStage;
                        onIdle();
                    }
                    break;
            }
        }

        // State-specific actions
        void onInit() {
            std::cout << "State changed to: Init" << std::endl;
            // Init pose Ex) 30 30 30 30 30 30
        }

        void onMocking() {
            std::cout << "State changed to: Mocking" << std::endl;
            // Synchronize target pose and input pose
        }

        void onIdle() {
            std::cout << "State changed to: Idle" << std::endl;
            // Additional actions when FSM enters the Idle state
        }

    };

}

#endif //VR_ARM_FSM_HPP
