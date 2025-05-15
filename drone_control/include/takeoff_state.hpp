#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: TAKEOFF");

        const Eigen::Vector3d fictual_home = Eigen::Vector3d({0.0, 0.0, 0.0});

        drone->toOffboardSync();
        drone->armSync();
        drone->setHomePosition(fictual_home);
                
        pos = drone->getLocalPosition();
        float target_height = *blackboard.get<float>("takeoff_height");
        initial_yaw = drone->getOrientation()[2];
        drone->log("Initial Yaw: " + std::to_string(initial_yaw));

        drone->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

        goal = Eigen::Vector3d({pos[0], pos[1], target_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos  = drone->getLocalPosition();

        if (i_%10==0){
            drone->log("Pos: {" + std::to_string(pos[0]) + ", " 
                        + std::to_string(pos[1]) + ", " + std::to_string(pos[2]) + "}");
        }
        i_++;

        if ((pos-goal).norm() < 0.15){
            return "TAKEOFF COMPLETED";
        }

        Eigen::Vector3d diff = goal - pos;
        Eigen::Vector3d little_goal = pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);
        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);
        
        return "";
    }

private:
    const float max_velocity = 1.0;
    Eigen::Vector3d pos, goal;
    Drone* drone;
    int i_ = 0;
    float initial_yaw;
};