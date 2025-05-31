#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <chrono>

class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: LANDING");

        pos = drone->getLocalPosition();

        start_time_ = std::chrono::steady_clock::now();

        drone->log("Descending for 10s.");

    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        pos = drone->getLocalPosition();
       
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();
        if (elapsed_time > 10){
            return "LANDED";
        }
        
        drone->setLocalVelocity(0.0, 0.0, 0.5, 0.0);
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        int counter = *blackboard.get<int>("counter");
        drone->log("Landed at base number "+ std::to_string(counter) + ". Position: " + std::to_string(pos[0]) + ", "+ std::to_string(pos[1]) + ", "+ std::to_string(pos[2]) + ".");
    }

private:
    Drone* drone;
    Eigen::Vector3d pos;
    std::chrono::steady_clock::time_point start_time_;
    
};