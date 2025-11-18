#ifndef SYSTEM_KG3FullInput_H
#define SYSTEM_KG3FullInput_H

#include <Eigen/Dense>
#include <vector>
#include <unsupported/Eigen/CXX11/Tensor>
#include <cmath>

#define pi 3.14159265358979323846
#define pi2 1.57079632679489661923

// Example class that represents the SystemKG3.
class SystemKG3FullInputMoreAggressive {
public:
    // All public member variables and constants

    // Link Lengths and heights
    const double h1 = 0.1564;
    const double h2 = 0.1284;
    const double l2 = 0.0054;
    const double h3 = 0.2104;
    const double l3 = 0.0064;
    const double h4 = 0.2104;
    const double l4 = 0.0064;
    const double h5 = 0.2084;
    const double l5 = 0.0064;
    const double h6 = 0.1059;
    const double h7 = 0.1059;
    const double h8 = 0.1815;

    // Location of centre of mass
    const double xc1 = -0.000648;
    const double yc1 = -0.000166;
    const double zc1 =  0.084487;

    const double xc2 = -0.000023;
    const double yc2 = -0.010364;
    const double zc2 = -0.073360;

    const double xc3 = -0.000044;
    const double yc3 = -0.099580;
    const double zc3 = -0.013278;

    const double xc4 = -0.000044;
    const double yc4 = -0.006641;
    const double zc4 = -0.117892;

    const double xc5 = -0.000018;
    const double yc5 = -0.075478;
    const double zc5 = -0.015006;

    const double xc6 =  0.000001;
    const double yc6 = -0.009432;
    const double zc6 = -0.063883;

    const double xc7 =  0.000001;
    const double yc7 = -0.045483;
    const double zc7 = -0.009650;

    const double xc8 = -0.000281;
    const double yc8 = -0.011402;
    const double zc8 = -0.077098 ;

    // Gravity
    const double g = 9.81;

    // Link mass [kg]
    const double m1 = 1.697;  // Link 1 mass [kg]
    const double m2 = 1.377;  // Link 2 mass [kg]
    const double m3 = 1.1636; // Link 3 mass [kg]
    const double m4 = 1.1636; // Link 4 mass [kg]
    const double m5 = 0.930;  // Link 5 mass [kg]
    const double m6 = 0.678;  // Link 6 mass [kg]
    const double m7 = 0.678;  // Link 7 mass [kg]
    const double m8 = 1.425;    // Link 8 mass [kg]

    // Joint damping
    const double d1 = 0.1;    
    const double d2 = 0.1;    
    const double d3 = 0.1;    
    const double d4 = 0.1;    
    const double d5 = 0.1;    
    const double d6 = 0.1;    
    const double d7 = 0.1;   
    const double d8 = 0.1;   

    // Link Inertia's 
    const Eigen::Matrix3d Ic11 = (Eigen::Matrix3d() << 
        0.004622, 0.000009, 0.000060,
        0.0,      0.004495, 0.000009,
        0.0,      0.0,      0.002079).finished();

    const Eigen::Matrix3d Ic22 = (Eigen::Matrix3d() << 
        0.004570, 0.000001, 0.000002,
        0.0,      0.004831, 0.000448,
        0.0,      0.0,      0.001409).finished();

    const Eigen::Matrix3d Ic33 = (Eigen::Matrix3d() << 
        0.011088, 0.000005, 0.0,
        0.0,      0.001072, -0.000691,
        0.0,      0.0,      0.011255).finished();

    const Eigen::Matrix3d Ic44 = (Eigen::Matrix3d() << 
        0.010932, 0.0,     -0.000007,
        0.0,      0.011127, 0.000606,
        0.0,      0.0,      0.001043).finished();

    const Eigen::Matrix3d Ic55 = (Eigen::Matrix3d() << 
        0.008147, -0.000001, 0.0,
        0.0,      0.00631,   -0.000500,
        0.0,      0.0,       0.008316).finished();

    const Eigen::Matrix3d Ic66 = (Eigen::Matrix3d() << 
        0.001596, 0.0,      0.0,
        0.0,      0.001607,  0.000256,
        0.0,      0.0,       0.000399).finished();

    const Eigen::Matrix3d Ic77 = (Eigen::Matrix3d() << 
        0.001641, 0.0,      0.0,
        0.0,      0.000410, -0.000278,
        0.0,      0.0,       0.001641).finished();

    const Eigen::Matrix3d Ic88 = (Eigen::Matrix3d() << 
        0.000692, 0.000003, 0.000003,
        0.0,      0.000542, 0.000118,
        0.0,      0.0,      0.001599).finished();

    // External Force
    Eigen::VectorXd Fext = Eigen::VectorXd::Zero(6); // External force vector (6x1)

    using Vec6d = Eigen::Matrix<double, 6, 1>;
    // State variables updated by various functions.
    Vec6d xe;  // End-effector pose (6x1)
    Eigen::Matrix<double, 6, 7> JA;  // Analytical Jacobian (6x7)

    // HA: a vector of matrices representing derivatives of JA w.r.t each joint variable.
    std::vector<Eigen::Matrix<double,6,7>> HA; // Each element: 6x7

    // Dynamic matrices
    Eigen::Matrix<double, 7, 7> M;   // Mass matrix (7x7)
    Eigen::Matrix<double, 7, 7> C;   // Coriolis matrix (7x7)
    Eigen::Matrix<double, 7, 7> D;   // Damping matrix (7x7)
    Eigen::Matrix<double, 7, 1> gq;  // Gravity torque vector (7x1)
    

    // Closed-loop (effector) matrices (6x6)
    Eigen::Matrix<double, 6, 6> Me;
    Eigen::Matrix<double, 6, 6> Ke;
    Eigen::Matrix<double, 6, 6> De;

    // 4x4 transformation matrices (modifiable)
    Eigen::Matrix4d A0c1;
    Eigen::Matrix4d A1c2;
    Eigen::Matrix4d A2c3;
    Eigen::Matrix4d A3c4;
    Eigen::Matrix4d A4c5;
    Eigen::Matrix4d A5c6;
    Eigen::Matrix4d A6c7;
    Eigen::Matrix4d A7c8;

    // Additional 4x4 transformation matrices (modifiable)
    Eigen::Matrix4d A0c2;
    Eigen::Matrix4d A0c3;
    Eigen::Matrix4d A0c4;
    Eigen::Matrix4d A0c5;
    Eigen::Matrix4d A0c6;
    Eigen::Matrix4d A0c7;
    Eigen::Matrix4d A0c8;

    // 6x7 Jacobian matrices (modifiable)
    Eigen::Matrix<double, 6, 7> Jc1;
    Eigen::Matrix<double, 6, 7> Jc2;
    Eigen::Matrix<double, 6, 7> Jc3;
    Eigen::Matrix<double, 6, 7> Jc4;
    Eigen::Matrix<double, 6, 7> Jc5;
    Eigen::Matrix<double, 6, 7> Jc6;
    Eigen::Matrix<double, 6, 7> Jc7;
    Eigen::Matrix<double, 6, 7> Jc8;

    // Derivative of homgenous transformation matrices w.r.t. q
    Eigen::Tensor<double, 3> dA01dq;
    Eigen::Tensor<double, 3> dA02dq;
    Eigen::Tensor<double, 3> dA03dq;
    Eigen::Tensor<double, 3> dA04dq;
    Eigen::Tensor<double, 3> dA05dq;
    Eigen::Tensor<double, 3> dA06dq;
    Eigen::Tensor<double, 3> dA07dq;
    Eigen::Tensor<double, 3> dA08dq;

    // Deriv of Transformation matrices to COM's w.r.t respect to q
    Eigen::Tensor<double, 3> dA0c2dq;
    Eigen::Tensor<double, 3> dA0c3dq;
    Eigen::Tensor<double, 3> dA0c4dq;
    Eigen::Tensor<double, 3> dA0c5dq;
    Eigen::Tensor<double, 3> dA0c6dq;
    Eigen::Tensor<double, 3> dA0c7dq;
    Eigen::Tensor<double, 3> dA0c8dq;

    // Derivatives of rotation matrices w.r.t. q
    Eigen::Tensor<double, 3> dR01dq;
    Eigen::Tensor<double, 3> dR02dq;
    Eigen::Tensor<double, 3> dR03dq;
    Eigen::Tensor<double, 3> dR04dq;
    Eigen::Tensor<double, 3> dR05dq;
    Eigen::Tensor<double, 3> dR06dq;
    Eigen::Tensor<double, 3> dR07dq;
    Eigen::Tensor<double, 3> dR08dq;
   
    // 6x7 matrices, 7 slices.
    Eigen::Tensor<double, 3> dJc1dq;
    Eigen::Tensor<double, 3> dJc2dq;
    Eigen::Tensor<double, 3> dJc3dq;
    Eigen::Tensor<double, 3> dJc4dq;
    Eigen::Tensor<double, 3> dJc5dq;
    Eigen::Tensor<double, 3> dJc6dq;
    Eigen::Tensor<double, 3> dJc7dq;
    Eigen::Tensor<double, 3> dJc8dq;

    //Gamme tensor for C matrix
    Eigen::Tensor<double, 3> Gamma; // Gamma tensor (7x7x7)

    // EE RPY angles 
    double xtheta;
    double ytheta;
    double xphi;
    double yphi;
    double xpsi;
    double ypsi;
    
    // 4x4 transformation matrices
    Eigen::Matrix4d A01;
    Eigen::Matrix4d A12;
    Eigen::Matrix4d A23;
    Eigen::Matrix4d A34;
    Eigen::Matrix4d A45;
    Eigen::Matrix4d A56;
    Eigen::Matrix4d A67;
    Eigen::Matrix4d A78;
    Eigen::Matrix4d A08;
    Eigen::Matrix4d A02;
    Eigen::Matrix4d A03;
    Eigen::Matrix4d A04;
    Eigen::Matrix4d A05;
    Eigen::Matrix4d A06;
    Eigen::Matrix4d A07;

    // 3x3 rotation matrices
    Eigen::Matrix3d R01;
    Eigen::Matrix3d R02;
    Eigen::Matrix3d R03;
    Eigen::Matrix3d R04;
    Eigen::Matrix3d R05;
    Eigen::Matrix3d R06;
    Eigen::Matrix3d R07;
    Eigen::Matrix3d R08;
    
    // 4x4 derivative matrices
    Eigen::Matrix4d dA01dq1;
    Eigen::Matrix4d dA12dq2;
    Eigen::Matrix4d dA23dq3;
    Eigen::Matrix4d dA34dq4;
    Eigen::Matrix4d dA45dq5;
    Eigen::Matrix4d dA56dq6;
    Eigen::Matrix4d dA67dq7;
    
    // 7x1 derivative vectors
    Eigen::Matrix<double, 7, 1> dyphidq;
    Eigen::Matrix<double, 7, 1> dxphidq;
    Eigen::Matrix<double, 7, 1> dythetadq;
    Eigen::Matrix<double, 7, 1> dx1thetadq;
    Eigen::Matrix<double, 7, 1> dx2thetadq;
    Eigen::Matrix<double, 7, 1> dypsidq;
    Eigen::Matrix<double, 7, 1> dxpsidq;


    // dMdq: vector of mass matrix derivatives (each 7x7) with respect to q(i)
    std::vector<Eigen::MatrixXd> dMdq; // size should be 7


    // Trajectory values
    Vec6d xestar; // Desried EE position
    Vec6d xestardt;  // Desried EE velocuty
    Vec6d xestarddt; // Desried EE acceleration

    // Constructor
    SystemKG3FullInputMoreAggressive();

    // Public methods
    Eigen::VectorXd computeInput(const Eigen::VectorXd &x, const Eigen::VectorXd &Fext, const Vec6d &xestar_in);
    void calcEffectorFKM(const Eigen::VectorXd &q);
    void calcEffectorAnalyticalJacobian(const Eigen::VectorXd &q);
    void calcEffectorAnalyticalHessian(const Eigen::VectorXd &q);
    void calcLinkGeometricJacobian(const Eigen::VectorXd &q);

private:
    // Private helper methods
    void calcLinkGeometricHessian(const Eigen::VectorXd &q);
    void calcMassMatrix(const Eigen::VectorXd &q);
    void calcCoriolisMatrix(const Eigen::VectorXd &q, const Eigen::VectorXd &dqdt);
    void calcDampingMatrix(const Eigen::VectorXd &dqdt);
    void calcGravityTorque(const Eigen::VectorXd &q);
    
    Eigen::MatrixXd compute_dMdt(const std::vector<Eigen::MatrixXd> &dMdq, const Eigen::VectorXd &dqdt);
    Eigen::MatrixXd compute_dJAdt(const std::vector<Eigen::Matrix<double,6,7>> &HA, const Eigen::VectorXd &dqdt);
    
    // Kinematics helper functions
    Eigen::Matrix4d kinematicsTranx(const double &q);
    Eigen::Matrix4d kinematicsTrany(const double &q);
    Eigen::Matrix4d kinematicsTranz(const double &q);
    Eigen::Matrix4d kinematicsRotx(const double &q);
    Eigen::Matrix4d kinematicsRotz(const double &q);
    Eigen::Matrix4d kinematicsRoty(const double &q);
    Eigen::Matrix3d kinematicsSkew(const Eigen::Vector3d &u);
    Eigen::Matrix4d kinematicsHatSE3(const Eigen::VectorXd &x);
    
    void setTensorSlice(Eigen::Tensor<double, 3>& tensor, int slice, const Eigen::Matrix4d& matrix);
    void setJacobianColumn(Eigen::Matrix<double,6,7>& J, int col, const Eigen::Vector3d& top, const Eigen::Vector3d& bottom);
    void setJacobianTensorSlice(Eigen::Tensor<double, 3>& tensor, int slice, const Eigen::Matrix<double,6,7>& matrix);
};

#endif // SYSTEM_KG3_H