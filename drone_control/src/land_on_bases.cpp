#include "fsm/fsm.hpp"

#include "takeoff_state.hpp"
#include "landing_state.hpp"
#include "goto_state.hpp"
#include "visiting_bases_state.hpp"
#include "going_home_state.hpp"
#include "initial_takeoff_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>


class LandOnBasesFSM : public fsm::FSM {
public:
    LandOnBasesFSM() : fsm::FSM({"ERROR", "FINISHED"}) {
        this->blackboard_set<Drone>("drone", new Drone());
        Drone* drone = blackboard_get<Drone>("drone");
        float takeoff_height = -2.5;
        std::vector<Eigen::Vector3d> bases = {
            Eigen::Vector3d(1.0, -4.0, 0.0),
            Eigen::Vector3d(2.0, -7.0, 0.0),
            Eigen::Vector3d(4.0, -5.0, 0.0),
            Eigen::Vector3d(6.0, -3.0, 0.0),
            Eigen::Vector3d(7.0, -1.0, 0.0)
        };
        
        
        int counter = 0;
        this->blackboard_set<std::vector<Eigen::Vector3d>>("bases", bases);
        this->blackboard_set<int>("counter", counter);
        this->blackboard_set<float>("takeoff_height", takeoff_height);
        this->blackboard_set<Eigen::Vector3d>("home_position", Eigen::Vector3d(0.0, 0.0, 0.0));
        this->blackboard_set<bool>("finished_bases", false);
        this->blackboard_set<float>("initial_yaw", drone->getOrientation()[2]);

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("VISITING BASES", std::make_unique<VisitBasesState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        this->add_transitions("INITIAL TAKEOFF", {{"TAKEOFF COMPLETED", "VISITING BASES"}, {"SEG FAULT", "ERROR"}});
        this->add_transitions("VISITING BASES", {{"ON BASE", "LANDING"}, {"SEG FAULT", "ERROR"}});
        this->add_transitions("TAKEOFF", {{"NO BASES LEFT", "RETURN HOME"}, {"BASES LEFT", "VISITING BASES"}, {"SEG FAULT", "ERROR"}});
        this->add_transitions("LANDING", {{"LANDED", "TAKEOFF"}, {"SEG FAULT", "ERROR"}});
        this->add_transitions("RETURN HOME", {{"AT HOME", "FINISHED"}, {"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("land_on_bases_node"), my_fsm() {
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
    LandOnBasesFSM my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
