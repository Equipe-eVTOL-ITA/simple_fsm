#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class VisitBasesState : public fsm::State {
public:
    VisitBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");

        if (drone == nullptr) return;
        drone->log("STATE: VISITING BASES");

        initial_yaw = *blackboard.get<float>("initial_yaw");
        auto bases = *blackboard.get<std::vector<Eigen::Vector3d>>("bases");
        counter = *blackboard.get<int>("counter");
        float takeoff_height = *blackboard.get<float>("takeoff_height");
        pos = drone->getLocalPosition();

        Eigen::Vector3d base = bases[counter];

        goal = Eigen::Vector3d(base[0], base[1], takeoff_height);
        max_velocity = 0.8;
       
    }

    std::string act(fsm::Blackboard &blackboard) override {

        pos = drone->getLocalPosition();

        if ((goal - pos).norm() < 0.10){
            drone->log("ARRIVED AT BASE");
            counter++;
            blackboard.set<int>("counter", counter);
            return "ON BASE";
        }
        
        //drone->log("POSITION: " + std::to_string(pos[0]) + ", "+ std::to_string(pos[1]) + ", "+ std::to_string(pos[2]) + ".");
        goal_diff = goal - pos;
        little_goal = pos + (goal_diff.norm() > max_velocity ? goal_diff.normalized() * max_velocity : goal_diff);

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);

        return "";

    }

private:
    Drone* drone;
    Eigen::Vector3d pos, goal, goal_diff, little_goal;
    float initial_yaw, max_velocity;
    int counter;
};