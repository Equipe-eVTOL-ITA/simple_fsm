#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <chrono>

class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
    }
    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        
        
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        //Publish base coordinates
        (void) blackboard;
        
    }

private:
    
};