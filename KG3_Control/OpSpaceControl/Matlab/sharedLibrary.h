#pragma once

#ifdef KINOVA_API_EXPORTS
    #define KINOVA_API __declspec(dllexport)
#else
    #define KINOVA_API __declspec(dllimport)
#endif

#include "mex.h"
#include <SessionManager.h>
#include <DeviceConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <ControlConfigClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <ActuatorCyclicClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <ActuatorCyclic.pb.h>

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001

class KINOVA_API KinovaApiWrapper {
public:
    static KinovaApiWrapper& getInstance();
    bool createApi(const char* ip_address);
    bool deleteApi();
    double* getExternalTorques(int* size);
    bool isConnected() const { return initialized; }
    bool sendTorques(const double* torques, int size);
    bool sendAngles(const double* positions, int size);
    struct TorqueCalibration {
        double current;
        double torque_gain;
        double torque_offset;
    };
    struct RobotState {
        std::vector<double> haptic_wrench;    // [Fx, Fy, Fz, Tx, Ty, Tz]
        std::vector<double> current_pos;       // [x, y, z]
        std::vector<double> current_rpy;       // [rx, ry, rz] in degrees
        std::vector<double> current_vel;       // [vx, vy, vz]
        std::vector<double> current_omega;     // [wx, wy, wz] in deg/s
        std::vector<double> twist_command;     // [vx, vy, vz, wx, wy, wz]
    };
    struct RobotStateNoQuat {
        std::vector<double> current_pos;       // [x, y, z]
        std::vector<double> current_rpy;       // [rx, ry, rz] in degrees
        std::vector<double> current_vel;       // [vx, vy, vz]
        std::vector<double> current_omega;     // [wx, wy, wz] in deg/s
        std::vector<double> twist_command;     // [vx, vy, vz, wx, wy, wz]
    };
    bool getTorqueCalibration(TorqueCalibration* calibration, int* size);
    bool setJointSpeeds(const double* speeds, int size);
    bool resetServoingMode();
    // Sending twist commands at high level approach
    bool sendTwistCommand(double linear_x, double linear_y, double linear_z, 
        double angular_x, double angular_y, double angular_z);
    bool getCurrentPose(double* pose); // Returns x,y,z,theta_x,theta_y,theta_z in array
    bool stopMotion(); // For emergency stops
    bool setTargetPosition(double x, double y, double z, double theta_x, double theta_y, double theta_z);
    bool getTargetPosition(double* position); // Returns stored position in the provided array
    bool KinovaApiWrapper::trackTargetPosition(double gain_linear, double gain_angular);
    // New methods for compliant motion
    bool enableCompliantTrajectoryTracking(bool enable, double force_threshold = 10.0, double torque_threshold = 5.0);
    bool getExternalCartesianForces(double* forces); // Get forces/torques at tool
    bool trackTargetPositionWithCompliance(double gain_linear, double gain_angular, 
                                          double compliance_linear = 0.5, double compliance_angular = 0.5);
     
    std::vector<double> getExternalCartesianForces(); // Get forces/torques at tool
    RobotState twistPDcontrol(const double* xDesired, const double* xdotDesired, int pedalState);
    RobotStateNoQuat twistPDcontrolNoQuat(const double* xDesired, const double* xdotDesired, int pedalState);
    bool controlGripperWithPedal(int pedalState);

private:
    // Constructor/Destructor
    KinovaApiWrapper();
    ~KinovaApiWrapper();
    
    bool gripperShouldClose;
    int previousPedalState;

    bool takeControlOfRobot();
    
    // Target position storage and mutex to ensure thread safety
    double m_target_position[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::mutex m_target_position_mutex; // To ensure thread safety 
    
    // Prevent copying
    KinovaApiWrapper(const KinovaApiWrapper&) = delete;
    KinovaApiWrapper& operator=(const KinovaApiWrapper&) = delete;
    
    k_api::TransportClientTcp* transport;
    k_api::RouterClient* router;
    k_api::SessionManager* session_manager;
    k_api::DeviceConfig::DeviceConfigClient* device_config;
    k_api::Base::BaseClient* base;
    k_api::TransportClientUdp* transport_rt;
    k_api::RouterClient* router_rt;
    k_api::SessionManager* session_manager_rt;
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic;
    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config;
    k_api::ControlConfig::ControlConfigClient* control_config;
    k_api::ActuatorCyclic::ActuatorCyclicClient* actuator_cyclic;
    
    bool initialized;
};