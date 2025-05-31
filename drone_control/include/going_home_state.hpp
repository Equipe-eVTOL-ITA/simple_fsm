#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: RETURN HOME");

        float takeoff_height = *blackboard.get<float>("takeoff_height");
        initial_yaw = *blackboard.get<float>("initial_yaw");
        pos = drone->getLocalPosition();
        max_velocity = 0.8;

        home = *blackboard.get<Eigen::Vector3d>("home");
        goal = Eigen::Vector3d(home[0], home[1], takeoff_height);

    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        pos = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.05) return "OVER HOME";

        goal_diff = goal - pos;
        little_goal = pos + (goal_diff.norm() > max_velocity ? goal_diff.normalized() * max_velocity : goal_diff);

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);

        return "";
    }


private:
    Drone* drone;
    Eigen::Vector3d home, goal, pos, goal_diff, little_goal;
    float max_velocity, initial_yaw;

};