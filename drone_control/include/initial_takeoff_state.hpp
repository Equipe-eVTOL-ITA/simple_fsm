#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <chrono>

class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");

        if (drone == nullptr) return;
        drone->log("STATE: INITIAL TAKEOFF");

        drone->toOffboardSync();
        drone->armSync();
        
        const Eigen::Vector3d fictual_home(1.2, -1.0, -0.6);
        blackboard.set<Eigen::Vector3d>("home", fictual_home);

        drone->setHomePosition(fictual_home);

        float takeoff_height = *blackboard.get<float>("takeoff_height");
        initial_yaw = *blackboard.get<float>("initial_yaw");

        pos = drone->getLocalPosition();
        goal = Eigen::Vector3d(pos[0], pos[1], takeoff_height);
        max_velocity = 0.8;

    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        pos = drone->getLocalPosition();
        if ((goal - pos).norm() < 0.10){
            drone->log("INITIAL TAKEOFF COMPLETED");
            return "TAKEOFF COMPLETED";
        }
        
        drone->log("POSITION: " + std::to_string(pos[0]) + ", "+ std::to_string(pos[1]) + ", "+ std::to_string(pos[2]) + ".");
        goal_diff = goal - pos;
        little_goal = pos + (goal_diff.norm() > max_velocity ? goal_diff.normalized() * max_velocity : goal_diff);

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);

        return "";

    }

private:
    Drone* drone;
    Eigen::Vector3d pos, goal, goal_diff, little_goal;
    float initial_yaw, max_velocity;
};