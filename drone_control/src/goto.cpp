#include "fsm/fsm.hpp"

#include "takeoff_state.hpp"
#include "landing_state.hpp"
#include "goto_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>


class GoToFSM : public fsm::FSM {
public:
    GoToFSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());

        float takeoff_height = -2.5;
        this->blackboard_set<float>("takeoff_height", takeoff_height);

        Eigen::Vector3d target_base(2.0, -2.0, 0.0);
        this->blackboard_set<Eigen::Vector3d>("target_base", target_base);

        // Adding states
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("GOTO", std::make_unique<GoToState>());

        // Initial Takeoff transitions
        this->add_transitions("TAKEOFF", {{"TAKEOFF COMPLETED", "GOTO"},{"SEG FAULT", "ERROR"}});

        // GoTo transitions
        this->add_transitions("GOTO", {{"ARRIVED AT POINT", "LANDING"},{"SEG FAULT", "ERROR"}});

        // Landing transitions
        this->add_transitions("LANDING", {{"LANDED", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("goto_node"), my_fsm() {
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
    GoToFSM my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
