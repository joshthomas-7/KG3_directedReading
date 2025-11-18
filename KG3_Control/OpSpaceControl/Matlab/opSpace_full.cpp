// This script implements a motion tracking operational space controller.
// The controller is an inverse dynamics controller.
// Integral action and non-integral action forms of the controller have been developed.
// It reads an operational space trajectory from "opspace_trajectory.csv". This csv must be in the same folder as the .exe of this script
// Velocity and acceleration tracking have not been tested as of 12/11/2025

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>

#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <RouterClient.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <SystemKG3.h>
#include <SystemKG3FullInput.h>

#define pi 3.14159265358979323846
#define pi2 1.57079632679489661923

#define PORT 10000
#define PORT_REAL_TIME 10001

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

float TIME_DURATION = 15.0f;
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

namespace k_api = Kinova::Api;

// ============================================================================
// FULL 6DOF OPERATIONAL SPACE TRAJECTORY DATA STRUCTURES
// ============================================================================

struct OpSpaceTrajectoryPoint {
    double time;
    Eigen::VectorXd xe;      // End-effector pose [x, y, z, phi, theta, psi] (6)
    Eigen::VectorXd dxe;     // End-effector velocity (6)
    Eigen::VectorXd ddxe;    // End-effector acceleration (6)
    
    OpSpaceTrajectoryPoint() : xe(6), dxe(6), ddxe(6) {}
};

class OpSpaceTrajectoryLoader {
private:
    std::vector<OpSpaceTrajectoryPoint> trajectory;
    
public:
    OpSpaceTrajectoryLoader() {}
    
    bool loadFromCSV(const std::string& filename, bool has_header = true) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return false;
        }
        
        std::string line;
        
        // Skip header if present
        if (has_header) {
            std::getline(file, line);
        }
        
        // Read trajectory data
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::vector<double> row;
            
            // Parse comma-separated values
            while (std::getline(ss, value, ',')) {
                try {
                    row.push_back(std::stod(value));
                } catch (const std::exception& e) {
                    std::cerr << "Warning: Failed to parse value: " << value << std::endl;
                    continue;
                }
            }
            
            // Check if we have the right number of columns (1 + 6 + 6 + 6 = 19)
            if (row.size() != 19) {
                std::cerr << "Warning: Skipping malformed line (expected 19 columns, got " 
                          << row.size() << ")" << std::endl;
                continue;
            }
            
            OpSpaceTrajectoryPoint point;
            point.time = row[0];
            
            // End-effector pose (columns 1-6: x, y, z, phi, theta, psi)
            for (int i = 0; i < 6; ++i) {
                point.xe(i) = row[1 + i];
            }
            
            // End-effector velocity (columns 7-12)
            for (int i = 0; i < 6; ++i) {
                point.dxe(i) = row[7 + i];
            }
            
            // End-effector acceleration (columns 13-18)
            for (int i = 0; i < 6; ++i) {
                point.ddxe(i) = row[13 + i];
            }
            
            trajectory.push_back(point);
        }
        
        file.close();
        std::cout << "Loaded " << trajectory.size() << " operational space trajectory points" << std::endl;
        return !trajectory.empty();
    }
    
    // Get trajectory point by index
    const OpSpaceTrajectoryPoint& getPoint(size_t index) const {
        if (index >= trajectory.size()) {
            return trajectory.back();
        }
        return trajectory[index];
    }
    
    // Get sample time
    double getSampleTime() const {
        if (trajectory.size() < 2) {
            return 0.001;
        }
        return trajectory[1].time - trajectory[0].time;
    }
    
    size_t size() const { return trajectory.size(); }
    double getDuration() const { 
        return trajectory.empty() ? 0.0 : trajectory.back().time; 
    }
};

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(int argc, char **argv)
{
    //---------------------------------------------------------
    // Initialization
    //---------------------------------------------------------
    std::string robot_ip = "192.168.1.10";
    std::string username = "admin";
    std::string password = "admin";

    auto error_callback = [](k_api::KError err){ 
        std::cout << "_________ callback error _________" << err.toString(); 
    };

    std::cout << "Creating transport objects..." << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(robot_ip, PORT);

    std::cout << "Creating real-time transport objects..." << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(robot_ip, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(username);
    create_session_info.set_password(password);
    create_session_info.set_session_inactivity_timeout(60000);
    create_session_info.set_connection_inactivity_timeout(2000);

    // Session manager service wrapper
    std::cout << "Creating sessions for communication..." << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    
    try {
        session_manager->CreateSession(create_session_info);
        session_manager_real_time->CreateSession(create_session_info);
    } catch (const k_api::KDetailedException& ex) {
        std::cerr << "Failed to create session(s): " << ex.what() << std::endl;
        delete session_manager_real_time;
        delete session_manager;
        delete router_real_time;
        delete transport_real_time;
        delete router;
        delete transport;
        return 1;
    }
    std::cout << "Sessions created successfully." << std::endl;

    // Create services objects
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    auto device_manager = new k_api::DeviceManager::DeviceManagerClient(router);

    //---------------------------------------------------------
    // Load Full 6DOF Operational Space Trajectory from CSV
    //---------------------------------------------------------
    OpSpaceTrajectoryLoader trajectory;
    std::string csv_filename = "opspace_trajectory.csv";
    
    // Option to specify file via command line
    if (argc > 1) {
        csv_filename = argv[1];
        std::cout << "Using trajectory file: " << csv_filename << std::endl;
    }
    
    if (!trajectory.loadFromCSV(csv_filename, true)) {
        std::cerr << "Failed to load trajectory from " << csv_filename << std::endl;
        std::cerr << "Exiting..." << std::endl;
        delete device_manager;
        delete actuator_config;
        delete base_cyclic;
        delete base;
        session_manager->CloseSession();
        session_manager_real_time->CloseSession();
        delete session_manager_real_time;
        delete session_manager;
        delete router_real_time;
        delete transport_real_time;
        delete router;
        delete transport;
        return 1;
    }
    
    std::cout << "\n=== FULL 6DOF OPERATIONAL SPACE CONTROL ===\n";
    std::cout << "Trajectory duration: " << trajectory.getDuration() << " seconds" << std::endl;
    std::cout << "Trajectory sample time: " << trajectory.getSampleTime() << " seconds" << std::endl;
    std::cout << "Number of trajectory points: " << trajectory.size() << std::endl;
    std::cout << "Tracking: Position (X, Y, Z) AND Orientation (φ, θ, ψ)\n" << std::endl;
    
    // Override TIME_DURATION to match trajectory duration
    TIME_DURATION = static_cast<float>(trajectory.getDuration() + 0.5);
    std::cout << "Setting control duration to " << TIME_DURATION << " seconds\n" << std::endl;

    //---------------------------------------------------------
    // Main logic - Setup before the loop
    //---------------------------------------------------------
    bool success = true;
    SystemKG3FullInput systemKG3;
    
    Eigen::VectorXd Fext(6);
    Fext << 0, 0, 0, 0, 0, 0;
    
    Eigen::VectorXd u(7);
    Eigen::VectorXd u_initial(7);
    u_initial << 0, 0, 0, 0, 0, 0, 0;
    
    Eigen::VectorXd tau_ext(7);
    tau_ext << 0, 0, 0, 0, 0, 0, 0;
    
    Eigen::VectorXd x(14);
    x << 0, 0, 0, 0, 0, 0, 0, 0, 0.2618, 3.1416, -2.2689, 0, 0.9599, 1.5708;
    
    // Initialize with first trajectory point (full 6DOF pose)
    Eigen::VectorXd xestar(6);
    OpSpaceTrajectoryPoint initial_point = trajectory.getPoint(0);
    xestar = initial_point.xe;
    
    std::cout << "Initial desired pose:" << std::endl;
    std::cout << "  Position:    [" << xestar(0) << ", " << xestar(1) << ", " << xestar(2) << "] m" << std::endl;
    std::cout << "  Orientation: [" << xestar(3) << ", " << xestar(4) << ", " << xestar(5) << "] rad" << std::endl;

    // OP SPACE CONTROLLER TUNEABLE GAINS
    double KP_aggro = 4000.0;                // Tuneable proportional gain for op space control (applies KP_true = KP_aggro*0.1)
    double KI_aggro = 12000.0;                // Tuneable integral gain for op space control (applies KI_true = KI_aggro*0.01)

    double KD_aggro = 50.0;                // Tuneable derivative gain for op space control

    Eigen::VectorXd integralError(6);
    integralError << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    double dt = 0.001;

    u_initial = systemKG3.computeInput(x, Fext, xestar);
    Eigen::MatrixXd Jc8_init(6, 7);
    Jc8_init = systemKG3.Jc8;
    std::cout << "Jacobian matrix Jc8_init: " << Jc8_init << std::endl;

    int maxIter = static_cast<int>(TIME_DURATION * 1000.0f);

    // Variables for storing logged data
    std::vector<Eigen::VectorXd> u_log;         u_log.reserve(maxIter);
    std::vector<Eigen::VectorXd> q_log;         q_log.reserve(maxIter);
    std::vector<Eigen::VectorXd> dqdt_log;      dqdt_log.reserve(maxIter);
    std::vector<Eigen::VectorXd> Fext_log;      Fext_log.reserve(maxIter);
    std::vector<Eigen::VectorXd> xestar_log;    xestar_log.reserve(maxIter);
    std::vector<Eigen::VectorXd> xe_log;        xe_log.reserve(maxIter);

    try {
        unsigned int actuator_count = base->GetActuatorCount().count();
        std::cout << "Robot has " << actuator_count << " actuators." << std::endl;
        unsigned int size = actuator_count;

        auto feedback = base_cyclic->RefreshFeedback();
        double* torques = new double[size];

        for(unsigned int i = 0; i < size; i++) {
            torques[i] = feedback.actuators(i).torque();
            std::cout << "Actuator " << i + 1 << " torque: " << torques[i] << " Nm" << std::endl;
        }
        std::cout << "Finished reading torque offsets." << std::endl;

        base->ClearFaults();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Read Initial External Wrench
        std::cout << "Reading initial external wrench..." << std::endl;
        Eigen::VectorXd Fext_init(6);
        try {
            feedback = base_cyclic->RefreshFeedback();
            Fext_init << feedback.base().tool_external_wrench_force_x(),
                         feedback.base().tool_external_wrench_force_y(),
                         feedback.base().tool_external_wrench_force_z(),
                         feedback.base().tool_external_wrench_torque_x(),
                         feedback.base().tool_external_wrench_torque_y(),
                         feedback.base().tool_external_wrench_torque_z();
            std::cout << "Initial External Wrench (Fext_init): " << Fext_init.transpose() << std::endl;
        } catch (const k_api::KDetailedException& ex) {
            std::cerr << "ERROR reading initial external wrench: " << ex.what() << std::endl;
            Fext_init.setZero(); 
        }

        // Set Servoing Mode to Low Level Servoing
        std::cout << "Setting servoing mode to LOW_LEVEL_SERVOING..." << std::endl;
        k_api::Base::ServoingModeInformation servoingModeCmd; 
        servoingModeCmd.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        try {
            base->SetServoingMode(servoingModeCmd);
        } catch (const k_api::KDetailedException& ex) {
             std::cerr << "ERROR sending SetServoingMode command: " << ex.what() << std::endl;
             throw;
        }

        // Set Actuators 1 through 6 to TORQUE mode
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        std::cout << "Setting actuators 1 through 6 to TORQUE mode..." << std::endl;
        for (uint32_t device_id = 1; device_id <= 6; ++device_id) {
            std::cout << "Setting actuator " << device_id << " to TORQUE mode..." << std::endl;
            try {
                actuator_config->SetControlMode(control_mode_message, device_id);
                std::cout << " -> Actuator " << device_id << " mode set command sent." << std::endl;
            } catch (const k_api::KDetailedException& ex) {
                std::cerr << "!!! ERROR setting actuator " << device_id << " to TORQUE mode: " << ex.what() << std::endl;
                success = false;
            } catch (const std::exception& ex) {
                std::cerr << "!!! Standard ERROR setting actuator " << device_id << " to TORQUE mode: " << ex.what() << std::endl;
                success = false;
            }
        }
        if (!success) {
             throw std::runtime_error("Failed to set one or more actuators to TORQUE mode.");
        }
        std::cout << "Actuators 1-7 set to TORQUE mode." << std::endl;
                
        // Initialize BaseCyclic Command & Feedback
        k_api::BaseCyclic::Feedback base_feedback;
        k_api::BaseCyclic::Command  base_command;

        for (unsigned int i = 0; i < actuator_count; ++i) {
            base_command.add_actuators();
        }

        base_feedback = base_cyclic->RefreshFeedback();

        //---------------------------------------------------------
        // Real-time loop - Full 6DOF tracking
        //---------------------------------------------------------
        std::cout << "\n=== Starting Full 6DOF Operational Space Control Loop ===" << std::endl;
        std::cout << "Duration: " << TIME_DURATION << " seconds" << std::endl;
        std::cout << "Tracking both position AND orientation\n" << std::endl;
        
        int timer_count = 0;
        int64_t now = 0;
        int64_t last = GetTickUs();
        
        // Track trajectory index
        size_t trajectory_index = 0;
        double trajectory_sample_time = trajectory.getSampleTime();
        int samples_per_trajectory_point = static_cast<int>(trajectory_sample_time / 0.001);
        
        std::cout << "Control loop: 1 kHz (1ms)" << std::endl;
        std::cout << "Trajectory sample time: " << trajectory_sample_time << " s" << std::endl;
        std::cout << "Updating pose every " << samples_per_trajectory_point << " control cycles\n" << std::endl;

        while (timer_count < (TIME_DURATION * 1000))
        {
            now = GetTickUs();

            if (now - last >= 1000)
            {
                try {
                    // Update position commands for ALL actuators
                    for (unsigned int i = 0; i < actuator_count; ++i) {
                         base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                    }

                    base_feedback = base_cyclic->Refresh(base_command, 0);

                    // Load the joint angles and velocities into state vector
                    Eigen::VectorXd q(7), dqdt(7);

                    for (unsigned int i = 0; i < actuator_count; ++i) {
                        if (i == 0) {
                            q(i) = base_feedback.actuators(i).position();
                            dqdt(i) = base_feedback.actuators(i).velocity();
                        }
                        else if (i == 3) {
                            q(i) =  base_feedback.actuators(i).position();
                            dqdt(i) = -base_feedback.actuators(i).velocity();
                        }
                        else if (i == 4) {
                            q(i) = base_feedback.actuators(i).position();
                            dqdt(i) = base_feedback.actuators(i).velocity();
                        }
                        else {
                            q(i) = base_feedback.actuators(i).position();
                            dqdt(i) = base_feedback.actuators(i).velocity();
                        }
                    }

                    q = q * pi / 180.0;
                    dqdt = dqdt * pi / 180.0;

                    // Read the torques from each of the actuators
                    for (unsigned int i = 0; i < actuator_count; ++i) {
                        torques[i] = base_feedback.actuators(i).torque();
                    }

                    x << dqdt, q;

                    // ============================================================
                    // Update desired 6DOF pose from trajectory
                    // ============================================================
                    if (timer_count % samples_per_trajectory_point == 0 && trajectory_index < trajectory.size()) {
                        OpSpaceTrajectoryPoint desired_point = trajectory.getPoint(trajectory_index);
                        xestar = desired_point.xe;
                        // Modify to read xestardt 
                        // xestardt = desired_point.dxe
                        // xestartddt = desired_point.ddxe
                        trajectory_index++;
                    }
                    // ============================================================

                    // Op Space Controller with no integral action
                    // u = systemKG3.opSpaceControl(x, xestar, KP_aggro, KD_aggro);

                    // Op Space Controller with integral action (does not include veleocity and acceleration tracking)
                    u = systemKG3.opSpaceControl_IA(x, xestar, KP_aggro, KD_aggro, KI_aggro);

                    // Op Space Controller with integral action (includes velcotiy and acceleration tracking). SystemKG3FullInput class will need to be modified accordingly
                    // u = systemKG3.opSpaceControl_IA(x, xestar, xestardt, xestarddt, KP_aggro, KD_aggro, KI_aggro);

                    
                    if (timer_count % 250 == 0) {
                        std::cout << "\n--- t = " << timer_count << " ms (trajectory point " << trajectory_index 
                                  << "/" << trajectory.size() << ") ---" << std::endl;
                        std::cout << "Torques: " << u.transpose() << std::endl;
                        
                        Eigen::VectorXd xe = systemKG3.xe;
                        
                        // Show position tracking
                        std::cout << "Position:" << std::endl;
                        std::cout << "  Desired: [" << xestar(0) << ", " << xestar(1) << ", " << xestar(2) << "]" << std::endl;
                        std::cout << "  Actual:  [" << xe(0) << ", " << xe(1) << ", " << xe(2) << "]" << std::endl;
                        Eigen::Vector3d pos_error = xe.head<3>() - xestar.head<3>();
                        std::cout << "  Error:   [" << pos_error(0)*1000 << ", " << pos_error(1)*1000 << ", " << pos_error(2)*1000 << "] mm" << std::endl;
                        
                        // Show orientation tracking
                        std::cout << "Orientation:" << std::endl;
                        std::cout << "  Desired: [" << xestar(3) << ", " << xestar(4) << ", " << xestar(5) << "] rad" << std::endl;
                        std::cout << "  Actual:  [" << xe(3) << ", " << xe(4) << ", " << xe(5) << "] rad" << std::endl;
                        Eigen::Vector3d ori_error = xe.tail<3>() - xestar.tail<3>();
                        std::cout << "  Error:   [" << ori_error(0) << ", " << ori_error(1) << ", " << ori_error(2) << "] rad" << std::endl;
                    }
                    
                    // Limit torques 
                    for (unsigned int i = 0; i < actuator_count; i++) {
                        Eigen::VectorXd g = systemKG3.gq;
                        if (u[i] > 25.0 || u[i] < -25.0) {
                            u[i] = g[i];
                            std::cout << "Gravity only control on joint " << i+1 << std::endl;
                        }
                    }

                    // Log the values
                    u_log.push_back(u);
                    q_log.push_back(q);
                    dqdt_log.push_back(dqdt);
                    xestar_log.push_back(xestar);
                    xe_log.push_back(systemKG3.xe);

                    // Calculate external torques
                    for (unsigned int i = 0; i < actuator_count; i++) {
                        tau_ext(i) = -torques[i] - u_initial(i);
                    }
                    
                    Eigen::Matrix<double,6,6> JJt = Jc8_init * Jc8_init.transpose();
                    Eigen::Matrix<double,6,1> F_tool = JJt.ldlt().solve(Jc8_init * tau_ext);
                    Fext_log.push_back(F_tool);

                    if (timer_count % 500 == 0) {
                        std::cout << "External Forces: " << Fext.transpose() << std::endl;
                    }
                    
                    // Apply Torque Commands to Actuators 1-6
                    for (unsigned int i = 0; i < 6; ++i) {
                        double scale_factor = 1.0;
                        
                        if (i == 3) {
                            scale_factor = 1.1;
                        }
                        else if (i == 5) {   
                            scale_factor = 1.1; 
                        } 
                        else if (i == 1) {
                            scale_factor = 1.1; 
                        } 
                        else if (i == 0) {
                            scale_factor = 1.0;
                        }
                        else if (i == 2) {
                            scale_factor = 1.0;
                        }
                        else if (i == 4) {
                            scale_factor = 2.1;
                        }

                        base_command.mutable_actuators(i)->set_torque_joint(static_cast<float>(scale_factor * u(i)));
                    }

                    // Increment Frame ID
                    base_command.set_frame_id(base_command.frame_id() + 1);
                    if (base_command.frame_id() > 65535)
                        base_command.set_frame_id(0);
                    for (unsigned int i = 0; i < actuator_count; ++i) {
                        base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
                    }

                    base_feedback = base_cyclic->Refresh(base_command, 0);

                } catch (const k_api::KDetailedException& ex) {
                    std::cerr << "Cyclic Refresh Error: " << ex.what() << std::endl;
                    success = false;
                    break;
                } catch (const std::exception& ex) {
                    std::cerr << "Standard exception in loop: " << ex.what() << std::endl;
                    success = false;
                    break;
                }

                timer_count++;
                last = now; 
            }
        }

        std::cout << "\nReal-time loop finished." << std::endl;

        // Cleanup after loop 
        std::cout << "Restoring default modes..." << std::endl;

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        std::cout << "Setting actuators 1 through 6 back to POSITION mode..." << std::endl;
        for (uint32_t device_id = 1; device_id <= 6; ++device_id) {
             try {
                 actuator_config->SetControlMode(control_mode_message, device_id);
                 std::this_thread::sleep_for(std::chrono::milliseconds(5));
             } catch (const k_api::KDetailedException& ex) {
                 std::cerr << "Warning: Failed to set actuator " << device_id << " back to POSITION mode: " << ex.what() << std::endl;
             } catch (const std::exception& ex) {
                 std::cerr << "Warning: Standard error setting actuator " << device_id << " back to POSITION mode: " << ex.what() << std::endl;
             }
        }
        std::cout << "Actuator modes restored." << std::endl;

        std::cout << "Pausing before changing servoing mode..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "Setting servoing mode back to SINGLE_LEVEL_SERVOING..." << std::endl;
        k_api::Base::ServoingModeInformation servoingMode;
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode); 
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "Modes restored." << std::endl;

    } catch (const k_api::KDetailedException& ex) {
        std::cerr << "API error during setup or cleanup: " << ex.what() << std::endl;
        success = false;
    } catch (const std::exception& ex) {
        std::cerr << "Standard exception during setup or cleanup: " << ex.what() << std::endl;
        success = false;
    }

    std::cout << "\nWriting logged data to CSV file..." << std::endl;

    std::ofstream csv("kinova_opspace_full_log.csv");
    if (!csv.is_open()) {
        std::cerr << "ERROR: could not open kinova_opspace_full_log.csv for writing.\n";
    } else {
        // Header for full 6DOF tracking
        csv << "iter";
        for (int j = 0; j < 7; ++j) csv << ",u" << (j+1);
        for (int j = 0; j < 7; ++j) csv << ",q" << (j+1);
        for (int j = 0; j < 7; ++j) csv << ",dq" << (j+1);
        for (int j = 0; j < 6; ++j) csv << ",xestar" << (j+1);
        for (int j = 0; j < 6; ++j) csv << ",xe" << (j+1);
        for (int j = 0; j < 6; ++j) csv << ",Fext" << (j+1);
        csv << "\n";

        int N = static_cast<int>(u_log.size());
        for (int i = 0; i < N; ++i) {
            csv << i;
            for (int j = 0; j < 7; ++j) csv << "," << u_log[i](j);
            for (int j = 0; j < 7; ++j) csv << "," << q_log[i](j);
            for (int j = 0; j < 7; ++j) csv << "," << dqdt_log[i](j);
            for (int j = 0; j < 6; ++j) csv << "," << xestar_log[i](j);
            for (int j = 0; j < 6; ++j) csv << "," << xe_log[i](j);
            for (int j = 0; j < 6; ++j) csv << "," << Fext_log[i](j);
            csv << "\n";
        }
        csv.close();
        std::cout << "CSV write complete (" << N << " rows)." << std::endl;
        std::cout << "Saved to: kinova_opspace_full_log.csv" << std::endl;
    }

    // Final Cleanup
    std::cout << "\nClosing sessions..." << std::endl;
    try {
        session_manager->CloseSession();
        session_manager_real_time->CloseSession();
    } catch (const k_api::KDetailedException& ex) {
        std::cerr << "Failed to close session(s) properly: " << ex.what() << std::endl;
    }
    std::cout << "Sessions closed." << std::endl;

    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    delete device_manager;
    delete actuator_config;
    delete base_cyclic;
    delete base;
    delete session_manager_real_time;
    delete session_manager;
    delete router_real_time;
    delete transport_real_time;
    delete router;
    delete transport;

    std::cout << "Cleanup complete." << std::endl;

    return success ? 0 : 1;
}