#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class GoToState : public fsm::State {
public:
    GoToState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: GoToState");

        Eigen::Vector3d target_base = *blackboard.get<Eigen::Vector3d>("target_base");
        float takeoff_height = *blackboard.get<float>("takeoff_height");

        goal = Eigen::Vector3d({target_base.x(), target_base.y(), takeoff_height});
        yaw = drone->getOrientation()[2];

        max_velocity = 0.8;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        pos = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.08) {
            return "ARRIVED AT POINT";
        }
        
        Eigen::Vector3d diff = goal - pos;
        Eigen::Vector3d little_goal = pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], yaw);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    Eigen::Vector3d pos, goal;
    Drone* drone;
    double max_velocity; // m/s
    float yaw;
};