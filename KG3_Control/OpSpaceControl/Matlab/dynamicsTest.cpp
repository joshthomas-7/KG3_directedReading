#include <iostream>
#include <string>
#include <vector>
#include <thread> // Required for std::this_thread::sleep_for
#include <chrono> // Required for std::chrono::seconds
#include <fstream>    // for std::ofstream


#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h> // Add this include
#include <RouterClient.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <DeviceManagerClientRpc.h> // Needed to get device IDs
#include <SystemKG3.h>
#include <SystemKG3FullInput.h>
// #include <SystemKG3FullInputMoreAggressive.h>

#define pi 3.14159265358979323846
#define pi2 1.57079632679489661923

#define PORT 10000
#define PORT_REAL_TIME 10001

// Add these lines
#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

float TIME_DURATION = 10.0f; // Duration of the example (seconds)
// Maximum allowed waiting time during actions 
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};
// ...existing code...

// Add this function from the example
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

int main(int argc, char **argv)
{
    //---------------------------------------------------------
    // Initialization
    //---------------------------------------------------------
    // Parse arguments for IP address ONLY
    std::string robot_ip = "192.168.1.10";
    // Hardcode username and password
    std::string username = "admin";
    std::string password = "admin";

    // Create API objects 
    auto error_callback = [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); };

    std::cout << "Creating transport objects..." << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(robot_ip, PORT);

    std::cout << "Creating real-time transport objects..." << std::endl;
    // Use UDP for real-time cyclic data as in the example
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(robot_ip, PORT_REAL_TIME);


    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(username);
    create_session_info.set_password(password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication..." << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time); // Session for real-time router
    try
    {
        session_manager->CreateSession(create_session_info);
        session_manager_real_time->CreateSession(create_session_info); // Create session for real-time router too
    }
    catch (const k_api::KDetailedException& ex)
    {
        std::cerr << "Failed to create session(s): " << ex.what() << std::endl;
        // Clean up before exiting
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
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time); // Use real-time router for cyclic
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    auto device_manager = new k_api::DeviceManager::DeviceManagerClient(router); // Keep if needed, otherwise remove


    //---------------------------------------------------------
    // Main logic - Setup before the loop
    //---------------------------------------------------------
    bool success = true;
    SystemKG3FullInput systemKG3; // Create an instance of SystemKG3
    Eigen::VectorXd Fext(6); // External forces 
    Fext << 0, 0, 0, 0, 0, 0; 
    Eigen::VectorXd u(7); // Control input vector
    Eigen::VectorXd u_initial(7);
    u_initial << 0, 0, 0, 0, 0, 0, 0; // Initialize control input vector
    Eigen::VectorXd tau_ext(7);
    tau_ext << 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd x(14);
    x << 0, 0, 0, 0, 0, 0, 0, 0, 0.2618, 3.1416, -2.2689, 0, 0.9599, 1.5708;
    Eigen::VectorXd xestar(6);
    // xestar << 0.579, -0.004, 0.43, pi/2, 0, pi/2;
    xestar << 0.579, -0.200, 0.43, pi/2, 0, pi/2;
    u_initial = systemKG3.computeInput(x, Fext, xestar); // Get control input from the model
    Eigen::MatrixXd Jc8_init(6, 7); // Jacobian matrix
    Jc8_init = systemKG3.Jc8;
    std::cout << "Jacobian matrix Jc8_init: " << Jc8_init << std::endl;

     // Compute the maximum number of iterations (TIME_DURATION is in seconds, loop is 1 kHz)
    int maxIter = static_cast<int>(TIME_DURATION * 1000.0f);

    // Variables for storing logged data
    std::vector<Eigen::VectorXd>   u_log;     u_log.reserve(maxIter);
    std::vector<Eigen::VectorXd>   q_log;     q_log.reserve(maxIter);
    std::vector<Eigen::VectorXd>   dqdt_log;  dqdt_log.reserve(maxIter);
    std::vector<Eigen::VectorXd>   Fext_log;  Fext_log.reserve(maxIter);

    try
    {
        // Get actuator count - useful for loops later
        unsigned int actuator_count = base->GetActuatorCount().count();
        std::cout << "Robot has " << actuator_count << " actuators." << std::endl;
        unsigned int size = actuator_count;

        // Get feedback using base cyclic client
        auto feedback = base_cyclic->RefreshFeedback();
        double* torques = new double[size]; // Use size directly

        // Read torque for each actuator directly from feedback
        for(unsigned int i = 0; i < size; i++) {
            torques[i] = feedback.actuators(i).torque();
            std::cout << "Actuator " << i + 1 << " torque: " << torques[i] << " Nm" << std::endl;
        }
        std::cout << "Finished reading torque offsets." << std::endl;

        // Clear faults first
        base->ClearFaults();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Give time to clear

        // *** Read Initial External Wrench ***
        std::cout << "Reading initial external wrench..." << std::endl;
        Eigen::VectorXd Fext_init(6);
        try {
            // Get a fresh feedback reading
            feedback = base_cyclic->RefreshFeedback();

            // Populate the vector [Fx, Fy, Fz, Tx, Ty, Tz]
            Fext_init << feedback.base().tool_external_wrench_force_x(),
                         feedback.base().tool_external_wrench_force_y(),
                         feedback.base().tool_external_wrench_force_z(),
                         feedback.base().tool_external_wrench_torque_x(),
                         feedback.base().tool_external_wrench_torque_y(),
                         feedback.base().tool_external_wrench_torque_z();

            std::cout << "Initial External Wrench (Fext_init): " << Fext_init.transpose() << std::endl;

        } catch (const k_api::KDetailedException& ex) {
            // Exception if reading the external wrench fails
            std::cerr << "ERROR reading initial external wrench: " << ex.what() << std::endl;
            Fext_init.setZero(); 
        }

        // 1. Set Servoing Mode to Low Level Servoing
        std::cout << "Setting servoing mode to LOW_LEVEL_SERVOING..." << std::endl;
        k_api::Base::ServoingModeInformation servoingModeCmd; 
        servoingModeCmd.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        try {
            base->SetServoingMode(servoingModeCmd);
        } catch (const k_api::KDetailedException& ex) {
             std::cerr << "ERROR sending SetServoingMode command: " << ex.what() << std::endl;
             throw; // Throw an error if setting to low level servoing mode fails
        }

        // --- Set Actuators 1 through 6 to TORQUE mode ---
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        std::cout << "Setting actuators 1 through 6 to TORQUE mode..." << std::endl;
        for (uint32_t device_id = 1; device_id <= 6; ++device_id) { // Loop through device IDs 1 to 6
            std::cout << "Setting actuator " << device_id << " to TORQUE mode..." << std::endl;
            try {
                actuator_config->SetControlMode(control_mode_message, device_id);
                std::cout << " -> Actuator " << device_id << " mode set command sent." << std::endl;

            } catch (const k_api::KDetailedException& ex) {
                std::cerr << "!!! ERROR setting actuator " << device_id << " to TORQUE mode: " << ex.what() << std::endl;
                success = false; 
                // if errors while setting actuators
            } catch (const std::exception& ex) { // Catch standard exceptions too
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
            base_command.add_actuators(); // Pre-populate the command structure
        }

        base_feedback = base_cyclic->RefreshFeedback(); // Get initial feedback

        //----------------------------------------  -----------------
        // Real-time loop
        //---------------------------------------------------------
        std::cout << "Starting real-time loop for " << TIME_DURATION << " seconds..." << std::endl;
        int timer_count = 0;
        int64_t now = 0;
        int64_t last = GetTickUs(); // Initialize last time

        while (timer_count < (TIME_DURATION * 1000)) // Loop runs for TIME_DURATION seconds at ~1kHz
        {
            now = GetTickUs();

            if (now - last >= 1000) // Check if 1000 microseconds (1ms) have passed
            {
                try
                {
                    // Update position commands for ALL actuators to prevent following errors
                    for (unsigned int i = 0; i < actuator_count; ++i) {
                         base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                    }

                    // Send command and get feedback for the next cycle
                    base_feedback = base_cyclic->Refresh(base_command, 0); // Timeout 0 for non-blocking

                    // Load the joint angles and velocities into state vector
                    Eigen::VectorXd q(7), dqdt(7);

                    for (unsigned int i = 0; i < actuator_count; ++i) {
                        if (i == 0){
                            q(i) = base_feedback.actuators(i).position(); // Adjust for Kinova's coordinate system
                            dqdt(i) = base_feedback.actuators(i).velocity(); // Adjust for Kinova's coordinate system
                        }
                        else if (i == 3){
                            q(i) =  base_feedback.actuators(i).position(); // Adjust for Kinova's coordinate system
                            dqdt(i) = -base_feedback.actuators(i).velocity(); // Adjust for Kinova's coordinate system
                        }
                        else if (i == 4) {
                            q(i) = base_feedback.actuators(i).position(); // Adjust for Kinova's coordinate system
                            dqdt(i) = base_feedback.actuators(i).velocity(); // Adjust for Kinova's coordinate system
                        }
                        else{
                            q(i) = base_feedback.actuators(i).position(); // Position in degrees
                            dqdt(i) = base_feedback.actuators(i).velocity(); // Velocity in deg/s
                        }
                    }

                    q = q * pi / 180.0; // Convert to radians
                    dqdt = dqdt * pi / 180.0; // Convert to radians/s


                    // Read the torques from each of the actuators
                    for (unsigned int i = 0; i < actuator_count; ++i) {
                        torques[i] = base_feedback.actuators(i).torque(); // Read torque from feedback
                    }

                    x << dqdt, q; // Concatenate dqdt and q into a single vector

                    u = systemKG3.computeInput(x, Fext, xestar); // Call computeInput method from SystemKG3
                    Eigen::VectorXd u_alt = systemKG3.computeInput_alt(x, Fext, xestar); // Call alternative controller to compare torque commands for the same inputs

                    // Now log the values
                    u_log.push_back(u);
                    q_log.push_back(q);
                    dqdt_log.push_back(dqdt);

                    // Calculate external torques
                    for (unsigned int i = 0; i < actuator_count; i++) {
                        tau_ext(i) = -torques[i] - u_initial(i); // Calculate external torques
                    }
                    // -- This is the external force estimation part -  I ended up removing it due to the unexpected behaviour it can have for the minor performance increase
                    //systemKG3.calcLinkGeometricJacobian(q);
                    Eigen::Matrix<double,6,6> JJt = Jc8_init * Jc8_init.transpose();
                    Eigen::Matrix<double,6,1> F_tool = JJt.ldlt().solve(Jc8_init * tau_ext); // Solve for F_tool using the pseudo-inverse of J 
                    Fext_log.push_back(F_tool); // Log the external forces
                    // Apply F_ext to next time step
                    // if (abs(F_tool(2)) > 10) {
                    //     Fext << 0, 0, -0.6*F_tool(2), 0, 0, 0;
                    // }
                    // else{
                    //     Fext << 0, 0, 0, 0, 0, 0;
                    // }
                    // if (abs(F_tool(0)) > 4){
                    //     Fext(1) = -0.4*F_tool(1); // Apply the force in Z direction
                    // }

                    if (timer_count % 500 == 0) {
                        std::cout << "External Forces at t = " << timer_count << "ms: " << Fext.transpose() << std::endl;
                    }
                    // --- Apply Torque Commands to Actuators 1-6 ---
                    for (unsigned int i = 0; i < 6; ++i) { // Loop through indices 0 to 5 (Actuators 1 to 6)
                        // Apply scaling factor if needed 
                        double scale_factor = 1.0; // Adjust if necessary
                        // You might want different factors per joint
                        if (i == 3){
                            scale_factor = 1.1;
                        }
                        else if (i == 5){   
                            scale_factor = 1.1; 
                        } 
                        else if (i == 1){
                            scale_factor = 1.1; 
                        } 
                        else if (i == 0){
                            scale_factor = 1.0;
                        }
                        else if (i == 2){
                            scale_factor = 1.0;
                        }
                        else if (i == 4){
                            scale_factor = 2.1; // Scaling factor to combat gear friction
                        }

                        base_command.mutable_actuators(i)->set_torque_joint(static_cast<float>(scale_factor * u(i)));
                    }

                    if (timer_count % 250 == 0) { // Adjust print frequency as needed
                        std::cout << "Calculated Dynamics Torques (u) at t = " << timer_count << "ms: " << u.transpose() << std::endl;
                        std::cout << "Alternative Torques (u_alt):" << u_alt.transpose() << std::endl;
                        std::cout << "x: " << x.transpose() << std::endl;
                        std::cout << "External Forces at t = " << timer_count << "ms: " << Fext.transpose() << std::endl;
                        // std::cout << "Desired End Effector Pose (xestar): " << xestar.transpose() << std::endl;
                    }

                    //  Increment Frame ID
                    base_command.set_frame_id(base_command.frame_id() + 1);
                    if (base_command.frame_id() > 65535)
                        base_command.set_frame_id(0);
                    for (unsigned int i = 0; i < actuator_count; ++i) {
                        base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
                    }

                    base_feedback = base_cyclic->Refresh(base_command, 0); // Timeout 0 for non-blockin


                }
                catch (const k_api::KDetailedException& ex)
                {
                    std::cerr << "Cyclic Refresh Error: " << ex.what() << std::endl;
                    // Decide how to handle errors (e.g., break loop, try to recover)
                    success = false;
                    break; // Exit loop on error for safety
                }
                 catch (const std::exception& ex)
                {
                    std::cerr << "Standard exception in loop: " << ex.what() << std::endl;
                    success = false;
                    break;
                }


                // Update timing
                timer_count++;
                last = now; 
            }
            else
            {

            }
        } // End of while loop

        std::cout << "Real-time loop finished." << std::endl;

    
        // Cleanup after loop 
        std::cout << "Restoring default modes..." << std::endl;

        // --- Set Actuators 1-6 back to POSITION mode ---
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        std::cout << "Setting actuators 1 through 6 back to POSITION mode..." << std::endl;
        for (uint32_t device_id = 1; device_id <= 6; ++device_id) { // Loop through device IDs 1 to 6
             try {
                 actuator_config->SetControlMode(control_mode_message, device_id);
                 std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Small delay
             } catch (const k_api::KDetailedException& ex) {
                 std::cerr << "Warning: Failed to set actuator " << device_id << " back to POSITION mode: " << ex.what() << std::endl;
                 // Continue cleanup even if one fails
             } catch (const std::exception& ex) {
                 std::cerr << "Warning: Standard error setting actuator " << device_id << " back to POSITION mode: " << ex.what() << std::endl;
             }
        }
        std::cout << "Actuator modes restored." << std::endl;


        // Add a delay before changing servoing mode 
        std::cout << "Pausing before changing servoing mode..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait 1 second

        // Now, set Servoing Mode back to SINGLE_LEVEL_SERVOING
        std::cout << "Setting servoing mode back to SINGLE_LEVEL_SERVOING..." << std::endl;
        k_api::Base::ServoingModeInformation servoingMode; // Re-declare if needed
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode); 
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait for mode change

        std::cout << "Modes restored." << std::endl;


    }
    catch (const k_api::KDetailedException& ex)
    {
        std::cerr << "API error during setup or cleanup: " << ex.what() << std::endl;
        success = false; // Mark as failed
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Standard exception during setup or cleanup: " << ex.what() << std::endl;
        success = false; // Mark as failed
    }

std::cout << "Writing logged data to CSV file..." << std::endl;

// Open an output file 
std::ofstream csv("kinova_log.csv");
if (!csv.is_open()) {
    std::cerr << "ERROR: could not open kinova_log.csv for writing.\n";
} else {
    csv << "iter";
    for (int j = 0; j < 7; ++j) csv << ",u"    << (j+1);
    for (int j = 0; j < 7; ++j) csv << ",q"    << (j+1);
    for (int j = 0; j < 7; ++j) csv << ",dq"   << (j+1);
    for (int j = 0; j < 6; ++j) csv << ",Fext" << (j+1);
    csv << "\n";

    int N = static_cast<int>(u_log.size());
    for (int i = 0; i < N; ++i) {
        csv << i;
        // write all 7 entries of u_log[i]
        for (int j = 0; j < 7; ++j) {
            csv << "," << u_log[i](j);
        }
        // write all 7 entries of q_log[i]
        for (int j = 0; j < 7; ++j) {
            csv << "," << q_log[i](j);
        }
        // write all 7 entries of dqdt_log[i]
        for (int j = 0; j < 7; ++j) {
            csv << "," << dqdt_log[i](j);
        }
        // write all 6 entries of fext_log[i]
        for (int j = 0; j < 6; ++j) {
            csv << "," << Fext_log[i](j);
        }

        csv << "\n";
    }
    csv.close();
    std::cout << "CSV write complete (" << N << " rows)." << std::endl;
}

    // Final Cleanup
    // Close API sessions
    std::cout << "Closing sessions..." << std::endl;
    try
    {
        session_manager->CloseSession();
        session_manager_real_time->CloseSession(); // Close real-time session too
    }
     catch (const k_api::KDetailedException& ex)
    {
        std::cerr << "Failed to close session(s) properly: " << ex.what() << std::endl;
        // Continue cleanup even if closing fails
    }
    std::cout << "Sessions closed." << std::endl;

    // Deactivate the routers and cleanly disconnect the transport layers
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy objects in reverse order of creation
    delete device_manager;
    delete actuator_config;
    delete base_cyclic;
    delete base;
    delete session_manager_real_time; // Delete real-time session manager
    delete session_manager;
    delete router_real_time;
    delete transport_real_time;
    delete router;
    delete transport;

    std::cout << "Cleanup complete." << std::endl;

    return success ? 0 : 1; // Return 0 on success, 1 on failure
}