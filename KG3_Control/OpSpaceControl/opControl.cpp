// This script is a temporarily holds the operational space motion control function
// It will be integrated in to dynamicsTest.cpp 

Eigen::VectorXd SystemKG3::computeInput(const Eigen::VectorXd &x, const Eigen::VectorXd &xestar) {
    // Extract joint velocities (dqdt) and positions (q) from the state vector
    Eigen::VectorXd dqdt = x.segment(0,7);
    Eigen::VectorXd q = x.segment(7,7);

    // Update system state and compute necessary matricies
    calcEffectorAnalyticalHessian(q);
    Eigen::VectorXd xe = this->xe;
    Eigen::MatrixXd JA = this->JA;

    // Helper function to compute dJAdt can be a private member function
    Eigen::MatrixXd dJAdt = compute_dJAdt(this->HA, dqdt);

    calcMassMatrix(q);
    Eigen::MatrixXd M = this->M;

    calcCoriolisMatrix(q, dqdt);
    Eigen::MatrixXd C = this->C;

    calcDampingMatrix(ddqdt);
    Eigen::MatrixXd D = this->D;

    calcGravityTorque(q);
    Eigen::VectorXd g = this->g;
    
    // Get the closed-loop effector matricies
    
    // Check for singularities
    if (M.fullPivLu().rank() < 5) {
        std::cerr << "Mass matrix M rank deficiency detected." << std::endl;
    }
    if (JA.fullPivLu().rank() < 5) {
        std::cerr << "Warning: Robot is in singular configuration." << std:endl;
        return Eigen::VectorXd::Zero(7);
    }

    // Control law



}