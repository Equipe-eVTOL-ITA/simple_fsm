#include "fsm/fsm.hpp"

#include "test_getter_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>


class TestGetterFSM : public fsm::FSM {
public:
    TestGetterFSM(std::shared_ptr<Drone> drone) : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);

        // Adding states
        this->add_state("TEST GETTER", std::make_unique<TestGetterState>());

        this->add_transitions("TEST GETTER", {{"TESTED", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM(std::shared_ptr<Drone> drone) : rclcpp::Node("getter_node"), drone_node_(drone) {

        fsm_ = std::make_unique<TestGetterFSM>(drone_node_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Run at approximately 20 Hz
            std::bind(&NodeFSM::executeFSM, this));
    }

    void executeFSM() {
        if (rclcpp::ok() && !fsm_->is_finished()) {
            fsm_->execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    std::shared_ptr<Drone> drone_node_;
    std::unique_ptr<TestGetterFSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    
    auto drone = std::make_shared<Drone>();
    auto fsm_node = std::make_shared<NodeFSM>(drone);
    
    executor.add_node(drone);
    executor.add_node(fsm_node);

    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}