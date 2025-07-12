#include "fsm/fsm.hpp"

#include "takeoff_state.hpp"
#include "landing_state.hpp"
#include "goto_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>


class GoToFSM : public fsm::FSM {
public:
    GoToFSM(std::shared_ptr<Drone> drone) : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);

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
    NodeFSM(std::shared_ptr<Drone> drone) : rclcpp::Node("goto_node"), drone_node_(drone) {

        fsm_ = std::make_unique<GoToFSM>(drone_node_);

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
    std::unique_ptr<GoToFSM> fsm_;
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
