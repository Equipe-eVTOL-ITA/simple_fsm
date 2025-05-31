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

        takeoff_height = *blackboard.get<float>("takeoff_height");
        initial_yaw = *blackboard.get<float>("initial_yaw");
        pos = drone->getLocalPosition();
        xy_pos = Eigen::Vector3d(pos[0], pos[1], takeoff_height);
        max_velocity = 0.8;

        home = *blackboard.get<Eigen::Vector3d>("home");
        goal = Eigen::Vector3d(home[0], home[1], takeoff_height);

    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
// jeito absolutament estupido de fazer isso, mas são 1h30 da manhã e eu quero dormir
        pos = drone->getLocalPosition();
        xy_pos = Eigen::Vector3d(pos[0], pos[1], takeoff_height);

        if ((xy_pos-goal).norm() < 0.10){
            goal_diff = home - pos;
            little_goal = pos + (goal_diff.norm() > max_velocity ? goal_diff.normalized() * max_velocity : goal_diff);
            drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);

            if((pos-home).norm() < 0.10) return "AT HOME";

            return "";
        
        }

        goal_diff = goal - pos;
        little_goal = pos + (goal_diff.norm() > max_velocity ? goal_diff.normalized() * max_velocity : goal_diff);

        drone->setLocalPosition(little_goal[0], little_goal[1], takeoff_height, initial_yaw);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override{
        (void)blackboard;
        pos = drone->getLocalPosition();
        drone->log("Disarming.");
        drone->disarmSync();
        
    }


private:
    Drone* drone;
    Eigen::Vector3d home, goal, pos, xy_pos, goal_diff, little_goal;
    float max_velocity, takeoff_height, initial_yaw;

};