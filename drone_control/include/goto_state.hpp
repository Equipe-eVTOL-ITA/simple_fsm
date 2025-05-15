#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class GoToState : public fsm::State {
public:
    GoToState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    
};