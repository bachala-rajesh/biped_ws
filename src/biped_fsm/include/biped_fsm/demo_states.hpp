#ifndef DEMO_STATES_HPP
#define DEMO_STATES_HPP

#include "yasmin/state.hpp"
#include "yasmin/blackboard.hpp"
#include <iostream>
#include <thread>
#include <chrono>

namespace biped_fsm {

class DemoInitState : public yasmin::State {
public:
    DemoInitState() : yasmin::State({"ready"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> /*blackboard*/) override {
        std::cout << "[INIT] Initializing robot sensors..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return "ready";
    }
};

class DemoPassiveState : public yasmin::State {
public:
    DemoPassiveState() : yasmin::State({"user_start"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> /*blackboard*/) override {
        std::cout << "[PASSIVE] Robot is passive. Waiting for user start..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return "user_start";
    }
};

class DemoStandbyState : public yasmin::State {
public:
    DemoStandbyState() : yasmin::State({"stop"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> /*blackboard*/) override {
        std::cout << "[STANDBY] Robot is in standby. Stopping demo..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return "stop";
    }
};

} // namespace biped_fsm

#endif