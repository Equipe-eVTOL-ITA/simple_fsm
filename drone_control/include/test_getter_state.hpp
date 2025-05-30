#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class TestGetterState : public fsm::State {
public:
    TestGetterState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: TEST GETTER");

        initial_time = std::chrono::high_resolution_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        // USAR OS GETTERS
        
        Eigen::Vector3d pos  = drone->getLocalPosition();
        Eigen::Vector3d orientation = drone->getOrientation();
        float speed = drone->getGroundSpeed();
        DronePX4::ARMING_STATE arming_state = drone->getArmingState();


        // PRINTAR INFORMACOES

        drone->log("Pos: {" + std::to_string(pos[0]) + ", " 
                    + std::to_string(pos[1]) + ", " + std::to_string(pos[2]) + "}");
                
        drone->log("Orientation: {" + std::to_string(orientation[0]) + ", " 
                    + std::to_string(orientation[1]) + ", " + std::to_string(orientation[2]) + "}");

        drone->log("Speed: " + std::to_string(speed));

        std::string arming_state_str = arming_state == DronePX4::ARMING_STATE::ARMED ? "ARMED" : "DISARMED";

        drone->log("Arming State: " + arming_state_str);


        // ACABAR ESTADO SE TIVER PASSADO 15 SEGUNDOS
        
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - initial_time).count();

        if (elapsed_time > 15) {
            return "TESTED";
        }
        
        return "";
    }

private:
    Drone* drone;
    std::chrono::time_point<std::chrono::high_resolution_clock> initial_time;
};