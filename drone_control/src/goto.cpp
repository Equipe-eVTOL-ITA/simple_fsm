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
