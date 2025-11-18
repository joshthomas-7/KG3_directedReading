#include "sharedLibrary.h"
#include <windows.h> 
#include <Eigen/Geometry>  

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

KinovaApiWrapper& KinovaApiWrapper::getInstance() {
    static KinovaApiWrapper instance;
    return instance;
}

KinovaApiWrapper::KinovaApiWrapper() : 
    transport(nullptr), router(nullptr), session_manager(nullptr),
    device_config(nullptr), base(nullptr), transport_rt(nullptr),
    router_rt(nullptr), session_manager_rt(nullptr), base_cyclic(nullptr),
    initialized(false),actuator_config(nullptr), control_config(nullptr),
    actuator_cyclic(nullptr),
    gripperShouldClose(true), 
    previousPedalState(0)     
    {}


KinovaApiWrapper::~KinovaApiWrapper() {
    if (initialized) {
        deleteApi();
    }
}

int64_t GetTickUs() {
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

bool KinovaApiWrapper::resetServoingMode() {
    if (!initialized || !base) {
        return false;
    }

    try {
        auto servoing_mode = k_api::Base::ServoingModeInformation();
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        return true;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Failed to reset servoing mode: %s\n", ex.what());
        return false;
    }
}

// phi = roll, theta = pitch, psi = yaw
// Converts RPY angles to quaternion
Eigen::Quaterniond rpyToQuat(double phi, double theta, double psi) {
    Eigen::AngleAxisd Rz(psi,   Eigen::Vector3d::UnitZ());  // yaw
    Eigen::AngleAxisd Ry(theta, Eigen::Vector3d::UnitY());  // pitch
    Eigen::AngleAxisd Rx(phi,   Eigen::Vector3d::UnitX());  // roll

    Eigen::Quaterniond q = Rz * Ry * Rx;

    return q.normalized();
};

bool KinovaApiWrapper::createApi(const char* ip_address) {
    try {
        // Check if already initialized
        if (initialized) {
            mexPrintf("API is already connected. Call deleteAPI() first.\n");
            return false;
        }

        std::string username = "admin";
        std::string password = "admin";

        auto error_callback = [](k_api::KError err){
            mexPrintf("Callback error: %s\n", err.toString().c_str());
        };

        transport = new k_api::TransportClientTcp();
        if (!transport) {
            mexPrintf("Failed to allocate transport\n");
            return false;
        }

        router = new k_api::RouterClient(transport, error_callback);
        if (!router) {
            delete transport;
            transport = nullptr;
            mexPrintf("Failed to allocate router\n");
            return false;
        }

        transport->connect(ip_address, PORT);

        // Set up session information
        auto create_session_info = k_api::Session::CreateSessionInfo();
        create_session_info.set_username(username);
        create_session_info.set_password(password);
        create_session_info.set_session_inactivity_timeout(60000);
        create_session_info.set_connection_inactivity_timeout(2000);

        // Create and start regular session
        session_manager = new k_api::SessionManager(router);
        session_manager->CreateSession(create_session_info);
        mexPrintf("Main session created successfully\n");

        // Create DeviceConfig client
        device_config = new k_api::DeviceConfig::DeviceConfigClient(router);

        // Create base client
        base = new k_api::Base::BaseClient(router);

        // Create UDP transport and router
        transport_rt = new k_api::TransportClientUdp();
        router_rt = new k_api::RouterClient(transport_rt, error_callback);
        transport_rt->connect(ip_address, PORT_REAL_TIME);

        // Create and start real-time session
        session_manager_rt = new k_api::SessionManager(router_rt);
        session_manager_rt->CreateSession(create_session_info);
        mexPrintf("Real-time session created successfully\n");

        // Create base cyclic client
        base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_rt);
        actuator_cyclic = new k_api::ActuatorCyclic::ActuatorCyclicClient(router_rt);
        actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);  // Use main router, not real-time
        control_config = new k_api::ControlConfig::ControlConfigClient(router); 

        // Verify both TCP and UDP connections
        try {
            auto actuator_count = base->GetActuatorCount();
            mexPrintf("TCP connection successful. Found %d actuators.\n", actuator_count.count());

            auto feedback = base_cyclic->RefreshFeedback();
            mexPrintf("UDP connection successful.\n");
        }
        catch (k_api::KDetailedException& ex) {
            // Clean up on failed connection
            deleteApi();
            mexPrintf("Failed to verify connection: %s\n", ex.what());
            return false;
        }

        initialized = true;
        mexPrintf("API and real-time cyclic client connected.\n");
        return true;
    }
    catch (std::exception& e) {
        deleteApi();
        mexPrintf("Error creating API: %s\n", e.what());
        return false;
    }
}

bool KinovaApiWrapper::deleteApi() {
    try {
        mexPrintf("Starting API cleanup...\n");

        if (!initialized) {
            mexPrintf("API is not connected. Nothing to clean up.\n");
            return true;
        }

        // First try to stop any ongoing motion
        try {
            if (base) {
                mexPrintf("Stopping any ongoing motion...\n");
                base->Stop();
                
                // Release control of the robot
                mexPrintf("Releasing control of robot...\n");
                base->Stop();
            }
        }
        catch (k_api::KDetailedException& ex) {
            mexPrintf("Warning: Error stopping motion: %s\n", ex.what());
        }

        // Close sessions first
        try {
            mexPrintf("Closing real-time session...\n");
            if (session_manager_rt) {
                session_manager_rt->CloseSession();
            }
            mexPrintf("Closing main session...\n");
            if (session_manager) {
                session_manager->CloseSession();
            }
        }
        catch (k_api::KDetailedException& ex) {
            mexPrintf("Warning: Error closing sessions: %s\n", ex.what());
        }

        // Deactivate routers and disconnect transports
        try {
            mexPrintf("Deactivating real-time connections...\n");
            if (router_rt) {
                router_rt->SetActivationStatus(false);
            }
            if (transport_rt) {
                transport_rt->disconnect();
            }

            mexPrintf("Deactivating main connections...\n");
            if (router) {
                router->SetActivationStatus(false);
            }
            if (transport) {
                transport->disconnect();
            }
        }
        catch (k_api::KDetailedException& ex) {
            mexPrintf("Warning: Error deactivating connections: %s\n", ex.what());
        }

        // Delete objects in reverse order of creation with error handling
        mexPrintf("Deleting objects...\n");
        try {
            if (base_cyclic) { 
                delete base_cyclic; 
                base_cyclic = nullptr;
                mexPrintf("Deleted base_cyclic\n");
            }
            if (actuator_cyclic) {
                delete actuator_cyclic;
                actuator_cyclic = nullptr;
                mexPrintf("Deleted actuator_cyclic\n");
            }
            
            if (session_manager_rt) { 
                delete session_manager_rt; 
                session_manager_rt = nullptr;
                mexPrintf("Deleted session_manager_rt\n");
            }
            
            if (router_rt) { 
                delete router_rt; 
                router_rt = nullptr;
                mexPrintf("Deleted router_rt\n");
            }
            
            if (transport_rt) { 
                delete transport_rt; 
                transport_rt = nullptr;
                mexPrintf("Deleted transport_rt\n");
            }
            
            if (base) { 
                delete base; 
                base = nullptr;
                mexPrintf("Deleted base\n");
            }
            
            if (device_config) { 
                delete device_config; 
                device_config = nullptr;
                mexPrintf("Deleted device_config\n");
            }
            
            if (session_manager) { 
                delete session_manager; 
                session_manager = nullptr;
                mexPrintf("Deleted session_manager\n");
            }
            
            if (router) { 
                delete router; 
                router = nullptr;
                mexPrintf("Deleted router\n");
            }
            
            if (transport) { 
                delete transport; 
                transport = nullptr;
                mexPrintf("Deleted transport\n");
            }
            if (actuator_config) {
                delete actuator_config;
                actuator_config = nullptr;
                mexPrintf("Deleted actuator_config\n");
            }
            if (control_config) {
                delete control_config;
                control_config = nullptr;
                mexPrintf("Deleted control_config\n");
            }
        }
        catch (std::exception& e) {
            mexPrintf("Warning: Exception during object deletion: %s\n", e.what());
        }

        initialized = false;
        mexPrintf("API disconnected and objects destroyed successfully.\n");
        return true;
    }
    catch (std::exception& e) {
        mexPrintf("Error during cleanup: %s\n", e.what());
        // Force cleanup even on error
        initialized = false;
        base_cyclic = nullptr;
        session_manager_rt = nullptr;
        router_rt = nullptr;
        transport_rt = nullptr;
        base = nullptr;
        device_config = nullptr;
        session_manager = nullptr;
        router = nullptr;
        transport = nullptr;
        return false;
    }
}


double* KinovaApiWrapper::getExternalTorques(int* size) {
    if (!initialized || !base_cyclic) {
        mexPrintf("API not initialized or base_cyclic client not available\n");
        *size = 0;
        return nullptr;
    }

    try {
        // Get actuator count
        unsigned int actuator_count = base->GetActuatorCount().count();
        *size = actuator_count;

        // Get feedback
        auto feedback = base_cyclic->RefreshFeedback();
        double* torques = new double[*size];

        // Read torque for each actuator directly from feedback
        for(int i = 0; i < *size; i++) {
            torques[i] = feedback.actuators(i).torque();  
            mexPrintf("Actuator %d torque: %.3f Nm\n", i + 1, torques[i]);
        }
        
        return torques;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Error reading torques: %s\n", ex.what());
        *size = 0;
        return nullptr;
    }
}

bool KinovaApiWrapper::sendAngles(const double* positions, int size) {

    if (!initialized || !base_cyclic || !base) {
        mexPrintf("API not initialized or clients not available\n");
        return false;
    }

    try {
        // Validate actuator count
        unsigned int actuator_count = base->GetActuatorCount().count();
        if (size != actuator_count) {
            mexPrintf("Position array size (%d) doesn't match actuator count (%d)\n", 
                     size, actuator_count);
            return false;
        }

        // Set the base in single level servoing mode
        auto servoing_mode = k_api::Base::ServoingModeInformation();
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        
        // Create the command
        auto command = k_api::Base::Action();
        command.set_name("Example angular action movement");
        command.set_application_data("");

        auto reach_joint_angles = command.mutable_reach_joint_angles();
        auto joint_angles = reach_joint_angles->mutable_joint_angles();

        // Set the joint angles
        for (int i = 0; i < size; i++) {
            auto joint_angle = joint_angles->add_joint_angles();
            joint_angle->set_joint_identifier(i);
            joint_angle->set_value(positions[i]);
            mexPrintf("Setting joint %d position to: %f degrees\n", i, positions[i]);
        }

        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            [&finish_promise](k_api::Base::ActionNotification notification) {
                const auto action_event = notification.action_event();
                if (action_event == k_api::Base::ActionEvent::ACTION_END ||
                    action_event == k_api::Base::ActionEvent::ACTION_ABORT) {
                    finish_promise.set_value(action_event);
                }
            },
            k_api::Common::NotificationOptions()
        );

        // Execute the action
        base->ExecuteAction(command);

        // Wait for action to complete
        const auto timeout = std::chrono::seconds(20);
        const auto status = finish_future.wait_for(timeout);
        base->Unsubscribe(promise_notification_handle);

        if (status != std::future_status::ready) {
            mexPrintf("Timeout waiting for position movement\n");
            return false;
        }

        auto event = finish_future.get();
        if (event != k_api::Base::ActionEvent::ACTION_END) {
            mexPrintf("Position movement failed or was aborted\n");
            return false;
        }

        mexPrintf("Position movement completed successfully\n");
        return true;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Failed to send positions: %s\n", ex.what());
        return false;
    }
}

// Function for checking the calibratio settings of the actuators
bool KinovaApiWrapper::getTorqueCalibration(TorqueCalibration* calibration, int* size) {
    if (!initialized || !actuator_config || !base) {
        mexPrintf("API not initialized or clients not available\n");
        return false;
    }
    
    try {
        // Get number of actuators
        unsigned int actuator_count = base->GetActuatorCount().count();
        mexPrintf("Reading torque calibration for all %d actuators...\n", actuator_count);
        
        // Loop through each actuator
        for (unsigned int i = 1; i <= actuator_count; i++) {
            auto torque_calibration = actuator_config->ReadTorqueCalibration(i);
            
            mexPrintf("\n=== Actuator %d ===\n", i);
            mexPrintf("Gain: %f\n", torque_calibration.gain());
            mexPrintf("Global Gain: %f\n", torque_calibration.global_gain());
            mexPrintf("Offset: %f\n", torque_calibration.offset());
            
        }
        
        return true;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Failed to read torque calibrations: %s\n", ex.what());
        return false;
    }
}


bool KinovaApiWrapper::setJointSpeeds(const double* speeds, int size) {
    if (!initialized || !base) {
        mexPrintf("API not initialized or base client not available\n");
        return false;
    }

    try {
        // Validate actuator count
        unsigned int actuator_count = base->GetActuatorCount().count();
        if (size != actuator_count) {
            mexPrintf("Speed array size (%d) doesn't match actuator count (%d)\n", 
                     size, actuator_count);
            return false;
        }

        // Create joint speeds command
        k_api::Base::JointSpeeds joint_speeds;
        
        // Set speed for each joint
        for (int i = 0; i < size; i++) {
            auto joint_speed = joint_speeds.add_joint_speeds();
            joint_speed->set_joint_identifier(i);
            joint_speed->set_value(speeds[i]);
            joint_speed->set_duration(0);  // 0 = continuous
        }

        // Send command
        base->SendJointSpeedsCommand(joint_speeds);
        
        // Check if we're stopping (all velocities are zero)
        bool all_zero = true;
        for (int i = 0; i < size; i++) {
            if (speeds[i] != 0.0) {
                all_zero = false;
                break;
            }
        }
        
        if (all_zero) {
            mexPrintf("All velocities are zero - motion stopped\n");
        }
        else {
            mexPrintf("Joint velocities sent - motion continuing\n");
        }
        
        return true;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Failed to send joint speeds: %s\n", ex.what());
        return false;
    }
}

bool KinovaApiWrapper::sendTwistCommand(double linear_x, double linear_y, double linear_z, 
    double angular_x, double angular_y, double angular_z) {
    if (!initialized || !base) {
    mexPrintf("API not initialized\n");
    return false;
    }

    try {
    // Create twist command
    auto command = k_api::Base::TwistCommand();

    // Set reference frame to base
    command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);

    command.set_duration(0);


    // Set the twist values
    auto twist = command.mutable_twist();
    twist->set_linear_x(linear_x);
    twist->set_linear_y(linear_y);
    twist->set_linear_z(linear_z);
    twist->set_angular_x(angular_x);
    twist->set_angular_y(angular_y);
    twist->set_angular_z(angular_z);

    // Send the command
    base->SendTwistCommand(command);
    return true;
    }
    catch (k_api::KDetailedException& ex) {
    mexPrintf("Error sending twist command: %s\n", ex.what());
    return false;
    }
}
bool KinovaApiWrapper::getCurrentPose(double* pose) {
    if (!initialized || !base_cyclic) {
        mexPrintf("API not initialized\n");
        return false;
    }
    
    try {
        auto feedback = base_cyclic->RefreshFeedback();
        
        // Fill pose array with current position and orientation
        pose[0] = feedback.base().tool_pose_x();
        pose[1] = feedback.base().tool_pose_y();
        pose[2] = feedback.base().tool_pose_z();
        pose[3] = feedback.base().tool_pose_theta_x();
        pose[4] = feedback.base().tool_pose_theta_y();
        pose[5] = feedback.base().tool_pose_theta_z();
        
        return true;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Error getting current pose: %s\n", ex.what());
        return false;
    }
}
bool KinovaApiWrapper::stopMotion() {
    if (!initialized || !base) {
        return false;
    }
    
    try {
        base->Stop();
        return true;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Error stopping motion: %s\n", ex.what());
        return false;
    }
}

std::vector<double> KinovaApiWrapper::getExternalCartesianForces() {
    if (!initialized || !base_cyclic) {
        mexPrintf("API not initialized\n");
        return {};
    };
    
    
    try {
        auto feedback = base_cyclic->RefreshFeedback();
        
        std::vector<double> forces(6);

        // Kortex provides estimated external wrench (force/torque) at the tool
        forces[0] = feedback.base().tool_external_wrench_force_x();
        forces[1] = feedback.base().tool_external_wrench_force_y();
        forces[2] = feedback.base().tool_external_wrench_force_z();
        forces[3] = feedback.base().tool_external_wrench_torque_x();
        forces[4] = feedback.base().tool_external_wrench_torque_y();
        forces[5] = feedback.base().tool_external_wrench_torque_z();
        
        return forces;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Error getting cartesian forces: %s\n", ex.what());
        return {};
    }
}

KinovaApiWrapper::RobotState KinovaApiWrapper::twistPDcontrol(const double* xDesired, const double* xdotDesired, int pedalState){
    if (!initialized || !base || !base_cyclic) {
        mexPrintf("API not initialized\n");
        return {};
    }
    RobotState state;
    try{
        // Conversion factor
        const double deg_to_rad = M_PI / 180.0;
        const double rad_to_deg = 180.0 / M_PI;

        // Get current pose
        auto feedback = base_cyclic->RefreshFeedback();
        // Current Pose (Linear + RPY in Radians)
        Eigen::Vector3d current_pos(feedback.base().tool_pose_x(),
        feedback.base().tool_pose_y(),
        feedback.base().tool_pose_z());
        // For plotting purposes
        Eigen::Vector3d current_rpy(feedback.base().tool_pose_theta_x(), feedback.base().tool_pose_theta_y(),
        feedback.base().tool_pose_theta_z());

        Eigen::Quaterniond current_quat = rpyToQuat(feedback.base().tool_pose_theta_x() * deg_to_rad,
                     feedback.base().tool_pose_theta_y() * deg_to_rad,
                     feedback.base().tool_pose_theta_z() * deg_to_rad);

        // Current Velocity (Linear + Angular in rad/s)
        Eigen::Vector3d current_vel_linear(feedback.base().tool_twist_linear_x(),
                                           feedback.base().tool_twist_linear_y(),
                                           feedback.base().tool_twist_linear_z());
        Eigen::Vector3d current_vel_angular(feedback.base().tool_twist_angular_x() * deg_to_rad,
                                            feedback.base().tool_twist_angular_y() * deg_to_rad,
                                            feedback.base().tool_twist_angular_z() * deg_to_rad);
        // For Plotting purposes                                   
        Eigen::Vector3d current_vel_angular_deg(feedback.base().tool_twist_angular_x(),
                                            feedback.base().tool_twist_angular_y(),
                                            feedback.base().tool_twist_angular_z());

        Eigen::Vector3d desired_pos(xDesired[0], xDesired[1], xDesired[2]);
        Eigen::Quaterniond desired_quat = rpyToQuat(xDesired[3], xDesired[4], xDesired[5]);
                                    
        Eigen::Vector3d desired_vel_linear(xdotDesired[0], xdotDesired[1], xdotDesired[2]);
        Eigen::Vector3d desired_vel_angular(xdotDesired[3], xdotDesired[4], xdotDesired[5]);

        // --- Calculate Errors ---
        // Linear Position Error
        Eigen::Vector3d error_pos = desired_pos - current_pos;

        // Orientation Error (using Quaternions)
        // q_error represents rotation from current_quat to desired_quat
        Eigen::Quaterniond q_error = desired_quat * current_quat.inverse();
        q_error.normalize(); // Ensure it's a unit quaternion

        // Convert error quaternion to axis-angle representation
        Eigen::AngleAxisd angle_axis_error(q_error);
        Eigen::Vector3d error_orientation = angle_axis_error.axis() * angle_axis_error.angle();
        // This vector's direction is the axis of rotation needed, magnitude is the angle (rad)

        // Linear Velocity Error
        Eigen::Vector3d error_vel_linear = desired_vel_linear - current_vel_linear;

        // Angular Velocity Error
        Eigen::Vector3d error_vel_angular = desired_vel_angular - current_vel_angular;

        // Gain values with some tunig done
        double Kp_linear = 1.5;  // Proportional gain for linear motion
        double Kd_linear = 0.7; // Derivative gain for linear motion
        double Kp_angular = 2;  // *** TUNE FOR QUATERNION ERROR *** (Start low, increase carefully)
        double Kd_angular = 0.5;

        // Calculate twist command components (linear in m/s, angular in rad/s)
        Eigen::Vector3d twist_linear_cmd = Kp_linear * error_pos + Kd_linear * error_vel_linear;
        Eigen::Vector3d twist_angular_cmd_rads = Kp_angular * error_orientation + Kd_angular * error_vel_angular;

        // Virtual Coupling Force Calculation (for Haptic Feedback) - didn't use but could be useful in future
        // Tuning Parameters for Virtual Spring-Damper
        double K_virt_linear = 20.0;  // Virtual linear stiffness (N/m)
        double D_virt_linear = 100.0;   // Virtual linear damping (N/(m/s))
        double K_virt_angular = 20.0;  // Virtual angular stiffness (Nm/rad)
        double D_virt_angular = 100.0;   // Virtual angular damping (Nm/(rad/s))

        // Calculate force/torque exerted BY the virtual coupling ON the robot
        Eigen::Vector3d coupling_force_on_robot = K_virt_linear * error_pos + D_virt_linear * error_vel_linear;
        Eigen::Vector3d coupling_torque_on_robot = K_virt_angular * error_orientation + D_virt_angular * error_vel_angular;

        // Force/Torque to be rendered on the handle (opposite direction)
        Eigen::Vector3d force_on_handle = -coupling_force_on_robot;
        Eigen::Vector3d torque_on_handle = -coupling_torque_on_robot;

        // Prepare the return vector [Fx, Fy, Fz, Tx, Ty, Tz]
        std::vector<double> haptic_feedback_wrench = {
            force_on_handle.x(), force_on_handle.y(), force_on_handle.z(),
            torque_on_handle.x(), torque_on_handle.y(), torque_on_handle.z()
        };

        // Prepare/send command
        auto command = k_api::Base::TwistCommand();
        command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);
        command.set_duration(0);  // Continuous control

        // Set the twist values, converting angular back to deg/s for the command
        auto twist = command.mutable_twist();
        twist->set_linear_x(twist_linear_cmd.x());
        twist->set_linear_y(twist_linear_cmd.y());
        twist->set_linear_z(twist_linear_cmd.z());
        twist->set_angular_x(twist_angular_cmd_rads.x() * rad_to_deg); 
        twist->set_angular_y(twist_angular_cmd_rads.y() * rad_to_deg); 
        twist->set_angular_z(twist_angular_cmd_rads.z() * rad_to_deg); 

        // Print debug info
        mexPrintf("Orientation Error (Axis*Angle): [%.3f, %.3f, %.3f]\n",
                  error_orientation.x(), error_orientation.y(), error_orientation.z());
        mexPrintf("Angular Vel Error (rad/s): [%.3f, %.3f, %.3f]\n",
                  error_vel_angular.x(), error_vel_angular.y(), error_vel_angular.z());
        mexPrintf("Twist Command (Sent as deg/s): Lin[%.3f, %.3f, %.3f] Ang[%.3f, %.3f, %.3f]\n",
                  twist->linear_x(), twist->linear_y(), twist->linear_z(),
                  twist->angular_x(), twist->angular_y(), twist->angular_z());


        // Call the gripper control function with the provided pedal state
        bool gripperSuccess = controlGripperWithPedal(pedalState);

        if (!gripperSuccess) {
            mexPrintf("Warning: Gripper command failed within twistPDcontrol loop.\n");
        }

        // Store variables to send back to MATLAB
        // Store current position
        state.current_pos = {
            current_pos.x(), current_pos.y(), current_pos.z()
        };

        // Store current RPY angles (in degrees)
        state.current_rpy = {
            current_rpy.x(), current_rpy.y(), current_rpy.z()
        };

        // Store current linear velocity
        state.current_vel = {
            current_vel_linear.x(), current_vel_linear.y(), current_vel_linear.z()
        };

        // Store current angular velocity (in deg/s)
        state.current_omega = {
            current_vel_angular_deg.x(), current_vel_angular_deg.y(), current_vel_angular_deg.z()
        };

        // Store twist command
        state.twist_command = {
            twist->linear_x(), twist->linear_y(), twist->linear_z(),
            twist->angular_x(), twist->angular_y(), twist->angular_z()
        };
        // Store wrench command
        state.haptic_wrench = haptic_feedback_wrench;
        
        // Send the command
        base->SendTwistCommand(command);

        return state;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Error in twistPDcontrol: %s\n", ex.what());
        return {}; // Returns empty vector on error
    }
    catch (const std::exception& ex) {
        mexPrintf("Standard exception in twistPDcontrol: %s\n", ex.what());
        return {}; // Returns empty vector on error
    }
}

// Define a constant for the gripper speed
const float GRIPPER_SPEED_CONSTANT = 0.1f; // Adjust as needed (positive opens, negative closes)

bool KinovaApiWrapper::controlGripperWithPedal(int pedalState) {
    if (!initialized || !base) {
        // Only need the base client
        mexPrintf("API not initialized or base client not available for gripper control\n");
        return {false};
    }

    // Ensure pedalState is either 0 or 1
    if (pedalState != 0 && pedalState != 1) {
        mexPrintf("Invalid pedal state received: %d. Expected 0 or 1.\n", pedalState);
        return false; // Or handle as an error
    }

    float targetSpeed = 0.0f; // Default to stop

    // Check for pedal release edge 
    if (pedalState == 0 && previousPedalState == 1) {
        // Pedal was just released, change direction for next press
        gripperShouldClose = !gripperShouldClose;
        targetSpeed = 0.0f; // Ensure gripper stops
    }
    // Check if pedal is currently pressed
    else if (pedalState == 1) {
        // Pedal is pressed, set speed based on the current direction mode
        targetSpeed = gripperShouldClose ? -GRIPPER_SPEED_CONSTANT : GRIPPER_SPEED_CONSTANT;
        if (previousPedalState == 0) {
             mexPrintf("Pedal pressed. Commanding gripper to %s.\n", gripperShouldClose ? "close" : "open");
        }
    }
    // Update the previous state for the next call *before* sending the command
    previousPedalState = pedalState;

    // Send Command 
    try {
        k_api::Base::GripperCommand gripper_command;
        gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);

        auto finger = gripper_command.mutable_gripper()->add_finger();
        finger->set_finger_identifier(1);
        finger->set_value(targetSpeed); // Set the calculated speed

        // Send the command
        base->SendGripperCommand(gripper_command);

        return true;
    }
    //cleanup
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Error sending gripper command: %s\n", ex.what());
        return false;
    }
    catch (std::exception& ex) {
        mexPrintf("Standard exception sending gripper command: %s\n", ex.what());
        return false;
    }
}
// This is a function to demonstrate how bad PD control is with no quaternions
KinovaApiWrapper::RobotStateNoQuat KinovaApiWrapper::twistPDcontrolNoQuat(const double* xDesired, const double* xdotDesired, int pedalState){
    if (!initialized || !base || !base_cyclic) {
        mexPrintf("API not initialized\n");
        return {};
    }
    RobotStateNoQuat state;
    try{
        // Conversion factor
        const double deg_to_rad = M_PI / 180.0;
        const double rad_to_deg = 180.0 / M_PI;

        // Get current pose
        auto feedback = base_cyclic->RefreshFeedback();
        double current_pose[6] = {
            feedback.base().tool_pose_x(),
            feedback.base().tool_pose_y(),
            feedback.base().tool_pose_z(),
            feedback.base().tool_pose_theta_x() * deg_to_rad,
            feedback.base().tool_pose_theta_y() * deg_to_rad, 
            feedback.base().tool_pose_theta_z() * deg_to_rad  
        };

        // Get current EE velocity
        double current_velocity_array[6] = {
            feedback.base().tool_twist_linear_x(),
            feedback.base().tool_twist_linear_y(),
            feedback.base().tool_twist_linear_z(),
            feedback.base().tool_twist_angular_x() * deg_to_rad, 
            feedback.base().tool_twist_angular_y() * deg_to_rad, 
            feedback.base().tool_twist_angular_z() * deg_to_rad  
        };

        // Assuming xDesired angular components are already in radians
        double pos_error[6];
        for (int i = 0; i < 6; i++) {
            pos_error[i] = xDesired[i] - current_pose[i];
            // Handle angle wrapping for angular errors 
            if (i >= 3) {
                 while (pos_error[i] > M_PI) pos_error[i] -= 2.0 * M_PI;
                 while (pos_error[i] <= -M_PI) pos_error[i] += 2.0 * M_PI;
            }
        }

        // Assuming xdotDesired angular components are already in rad/s
        double vel_error[6];
        for (int i = 0; i < 6; i++) {
            vel_error[i] = xdotDesired[i] - current_velocity_array[i];
        }

        // Create twist command with PD control
        auto command = k_api::Base::TwistCommand();
        command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);
        command.set_duration(0); // Continurous


        double twist_command_rads[6]; // Command calculated in rad/s for angular
        // Tuning Parameters 
        double Kp_linear = 0.75;  
        double Kd_linear = 0.375; 
        double Kp_angular = 1.0;  
        double Kd_angular = 0.1; 

        // Linear components 
        for (int i = 0; i < 3; i++) {
            twist_command_rads[i] = Kp_linear * pos_error[i] + Kd_linear * vel_error[i];
        }

        // Angular components 
        for (int i = 3; i < 6; i++) {
            twist_command_rads[i] = Kp_angular * pos_error[i] + Kd_angular * vel_error[i];
        }

        // Set the twist values, converting angular back to deg/s for the command
        auto twist = command.mutable_twist();
        twist->set_linear_x(twist_command_rads[0]);
        twist->set_linear_y(twist_command_rads[1]);
        twist->set_linear_z(twist_command_rads[2]);
        twist->set_angular_x(twist_command_rads[3] * rad_to_deg); // Convert back to deg/s
        twist->set_angular_y(twist_command_rads[4] * rad_to_deg); 
        twist->set_angular_z(twist_command_rads[5] * rad_to_deg); 

        // Print debug info 
        mexPrintf("Current Pose (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
            current_pose[0], current_pose[1], current_pose[2],
            current_pose[3], current_pose[4], current_pose[5]);

        mexPrintf("Desired Pose (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
            xDesired[0], xDesired[1], xDesired[2],
            xDesired[3], xDesired[4], xDesired[5]);

        mexPrintf("Twist Command (Sent as deg/s): Lin[%.3f, %.3f, %.3f] Ang[%.3f, %.3f, %.3f]\n",
            twist->linear_x(), twist->linear_y(), twist->linear_z(),
            twist->angular_x(), twist->angular_y(), twist->angular_z());


        // --- Call Gripper Control ---.
        bool gripperSuccess = controlGripperWithPedal(pedalState);
        if (!gripperSuccess) {
            mexPrintf("Warning: Gripper command failed within twistPDcontrol loop.\n");
        }
        // Store current position
        state.current_pos = {
            current_pose[0], current_pose[1], current_pose[2]
        };

        // Store current RPY angles (in degrees)
        state.current_rpy = {
            current_pose[3], current_pose[4], current_pose[5]
        };

        // Store current linear velocity
        state.current_vel = {
            current_velocity_array[0], current_velocity_array[1], current_velocity_array[2]
        };

        // Store current angular velocity (in deg/s)
        state.current_omega = {
            current_velocity_array[3], current_velocity_array[4], current_velocity_array[5]
        };
        // Store twist command
        state.twist_command = {
            twist_command_rads[0], twist_command_rads[1], twist_command_rads[2],
           twist_command_rads[3], twist_command_rads[4], twist_command_rads[5]
        };

        // Send the command
        base->SendTwistCommand(command);
        return state;
    }
    catch (k_api::KDetailedException& ex) {
        mexPrintf("Error in twistPDcontrol: %s\n", ex.what());
        return {};
    }
}


