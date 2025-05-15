#include "fsm/fsm.hpp"

#include "takeoff_state.hpp"
#include "landing_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>


class TakeoffLandingFSM : public fsm::FSM {
public:
    TakeoffLandingFSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        Drone* drone = blackboard_get<Drone>("drone");
        drone->log("Bla");

        float takeoff_height = -2.5;
        this->blackboard_set<float>("takeoff_height", takeoff_height);

        // Adding states
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        // Initial Takeoff transitions
        this->add_transitions("TAKEOFF", {{"TAKEOFF COMPLETED", "LANDING"},{"SEG FAULT", "ERROR"}});

        // Landing transitions
        this->add_transitions("LANDING", {{"LANDED", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("takeoff_landing_node"), my_fsm() {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Run at approximately 20 Hz
            std::bind(&NodeFSM::executeFSM, this));
    }

    void executeFSM() {
        if (rclcpp::ok() && !my_fsm.is_finished()) {
            my_fsm.execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    TakeoffLandingFSM my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
