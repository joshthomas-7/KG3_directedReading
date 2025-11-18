#include "SystemKG3FullInput.h"
#include <iostream>
#include <cmath>

// #ifndef M_PI
//     #define M_PI = 3.14159265358979323846
// #endif

static const double M_PI = 3.14159265358979323846;


SystemKG3FullInput::SystemKG3FullInput() : dA01dq(4, 4, 7), dA02dq(4, 4, 7), dA03dq(4, 4, 7), dA04dq(4, 4, 7), dA05dq(4, 4, 7), dA06dq(4, 4, 7), dA07dq(4, 4, 7), dA08dq(4, 4, 7), dJc1dq(6, 7, 7), dJc2dq(6, 7, 7), dJc3dq(6, 7, 7),
                                           dJc4dq(6, 7, 7), dJc5dq(6, 7, 7), dJc6dq(6, 7, 7), dJc7dq(6, 7, 7), dJc8dq(6, 7, 7), Me(6, 6), Ke(6, 6), De(6, 6), M(7, 7), C(7, 7), D(7, 7), gq(7), xe(6), JA(6, 7), HA(7, Eigen::Matrix<double, 6, 7>::Zero()),
                                           dA0c2dq(4, 4, 7), dA0c3dq(4, 4, 7), dA0c4dq(4, 4, 7), dA0c5dq(4, 4, 7), dA0c6dq(4, 4, 7), dA0c7dq(4, 4, 7), dA0c8dq(4, 4, 7), dMdq(7, Eigen::Matrix<double, 7, 7>::Zero()), dR01dq(3, 3, 7), dR02dq(3, 3, 7),
                                           dR03dq(3, 3, 7), dR04dq(3, 3, 7), dR05dq(3, 3, 7), dR06dq(3, 3, 7), dR07dq(3, 3, 7), dR08dq(3, 3, 7), Gamma(7, 7, 7)
{
    Me.setZero();
    Ke.setZero();
    De.setZero();
    M.setZero();
    C.setZero();
    D.setZero();
    Me.diagonal() << 1, 1, 1, 1, 1, 1;
    Ke.diagonal() << 64, 64, 64, 64, 64, 64; // Define the closed-loop charcterisistics of the system
    De.diagonal() << 16, 16, 16, 16, 16, 16;

    // KP.diagonal() << 16, 16, 16, 16, 16, 16;    // Aggressive inverse dynamics operational space PD controller
    // KD.diagonal() << 8, 8, 8, 8, 8, 8;

    KP.diagonal() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1; // Less aggressive inverse dynamics operational space PD controller
    KD.diagonal() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    KI.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

    // KP.diagonal() << 0.1, 0.1, 0.1, 1, 1, 1; // Less aggressive inverse dynamics operational space PD controller
    // KD.diagonal() << 0.1, 0.1, 0.1, 1, 1, 1;
    // KI.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;

    integral_max << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;   // Saturation for the integral action
    integralError << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    KP_grav.diagonal() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1; // Gravity compensation controller gains
    KD_grav.diagonal() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;


    xestar << 0.579, -0.004, 0.43, pi / 2, 0, pi / 2; // Initialise desired enf-effector pos,vel and acc
    qstardt.setZero();
    qstarddt.setZero();
    xestardt.setZero();
    xestarddt.setZero();
    dJc1dq.setZero();
    dJc2dq.setZero();
    dJc3dq.setZero();
    dJc4dq.setZero();
    dJc5dq.setZero();
    dJc6dq.setZero();
    dJc7dq.setZero();
    dJc8dq.setZero();
    dA01dq.setZero();
    dA02dq.setZero();
    dA03dq.setZero();
    dA04dq.setZero();
    dA05dq.setZero();
    dA06dq.setZero();
    dA07dq.setZero();
    dA08dq.setZero();
    dR01dq.setZero();
    dR02dq.setZero();
    dR03dq.setZero();
    dR04dq.setZero();
    dR05dq.setZero();
    dR06dq.setZero();
    dR07dq.setZero();
    dR08dq.setZero();
    dA0c2dq.setZero();
    dA0c3dq.setZero();
    dA0c4dq.setZero();
    dA0c5dq.setZero();
    dA0c6dq.setZero();
    dA0c7dq.setZero();
    dA0c8dq.setZero();
    Gamma.setZero();
    gq.setZero();
    R01.setZero();
    R02.setZero();
    R03.setZero();
    R04.setZero();
    R05.setZero();
    R06.setZero();
    R07.setZero();
    R08.setZero();
    HA.resize(7);
    for (int i = 0; i < 7; ++i)
    {
        HA[i] = Eigen::Matrix<double, 6, 7>::Zero();
    }
}

void SystemKG3FullInput::calcEffectorAnalyticalHessian(const Eigen::VectorXd &q)
{
    // Update xe, JA, and HA based on current q.
    // Call EffectorFKM
    calcEffectorFKM(q);
    // Call calcEffectorAnalyticalJacobian
    calcEffectorAnalyticalJacobian(q);

    Eigen::MatrixXd e6(6, 1);
    e6 << 0, 0, 0, 0, 0, 1;
    // Second derivative of transf matrices with respect to q1,2,3..7
    Eigen::Matrix4d ddA01dq1dq1 = kinematicsTranz(h1) * kinematicsRotx(pi) * kinematicsHatSE3(e6) * (kinematicsHatSE3(e6) * kinematicsRotz(q(0)));
    Eigen::Matrix4d ddA12dq2dq2 = kinematicsTranz(-h2) * kinematicsTrany(l2) * kinematicsRotx(pi2) * kinematicsHatSE3(e6) * (kinematicsHatSE3(e6) * kinematicsRotz(q(1)));
    Eigen::Matrix4d ddA23dq3dq3 = kinematicsTrany(-h3) * kinematicsTranz(-l3) * kinematicsRotx(-pi2) * kinematicsHatSE3(e6) * (kinematicsHatSE3(e6) * kinematicsRotz(q(2)));
    Eigen::Matrix4d ddA34dq4dq4 = kinematicsTranz(-h4) * kinematicsTrany(l4) * kinematicsRotx(pi2) * kinematicsHatSE3(e6) * (kinematicsHatSE3(e6) * kinematicsRotz(q(3)));
    Eigen::Matrix4d ddA45dq5dq5 = kinematicsTrany(-h5) * kinematicsTranz(-l5) * kinematicsRotx(-pi2) * kinematicsHatSE3(e6) * (kinematicsHatSE3(e6) * kinematicsRotz(q(4)));
    Eigen::Matrix4d ddA56dq6dq6 = kinematicsTranz(-h6) * kinematicsRotx(pi2) * kinematicsHatSE3(e6) * (kinematicsHatSE3(e6) * kinematicsRotz(q(5)));
    Eigen::Matrix4d ddA67dq7dq7 = kinematicsTrany(-h7) * kinematicsRotx(-pi2) * kinematicsHatSE3(e6) * (kinematicsHatSE3(e6) * kinematicsRotz(q(6)));

    Eigen::Tensor<double, 4> ddA08dqdq(4, 4, 7, 7); // dimensions: 4x4 matrices, 7 inner slices, 7 outer slices

    for (int l = 0; l < 7; ++l)
    {
        for (int k = 0; k < 7; ++k)
        {
            Eigen::Matrix4d term; // This will hold the computed 4x4 product.
            if (l == 0)
            {
                // Group 0: derivative in the first factor.
                switch (k)
                {
                case 0:
                    term = ddA01dq1dq1 * A12 * A23 * A34 * A45 * A56 * A67 * A78;
                    break;
                case 1:
                    term = dA01dq1 * dA12dq2 * A23 * A34 * A45 * A56 * A67 * A78;
                    break;
                case 2:
                    term = dA01dq1 * A12 * dA23dq3 * A34 * A45 * A56 * A67 * A78;
                    break;
                case 3:
                    term = dA01dq1 * A12 * A23 * dA34dq4 * A45 * A56 * A67 * A78;
                    break;
                case 4:
                    term = dA01dq1 * A12 * A23 * A34 * dA45dq5 * A56 * A67 * A78;
                    break;
                case 5:
                    term = dA01dq1 * A12 * A23 * A34 * A45 * dA56dq6 * A67 * A78;
                    break;
                case 6:
                    term = dA01dq1 * A12 * A23 * A34 * A45 * A56 * dA67dq7 * A78;
                    break;
                }
            }
            else if (l == 1)
            {
                // Group 1: derivative in the second factor.
                switch (k)
                {
                case 0:
                    term = dA01dq1 * dA12dq2 * A23 * A34 * A45 * A56 * A67 * A78;
                    break;
                case 1:
                    term = A01 * ddA12dq2dq2 * A23 * A34 * A45 * A56 * A67 * A78;
                    break;
                case 2:
                    term = A01 * dA12dq2 * dA23dq3 * A34 * A45 * A56 * A67 * A78;
                    break;
                case 3:
                    term = A01 * dA12dq2 * A23 * dA34dq4 * A45 * A56 * A67 * A78;
                    break;
                case 4:
                    term = A01 * dA12dq2 * A23 * A34 * dA45dq5 * A56 * A67 * A78;
                    break;
                case 5:
                    term = A01 * dA12dq2 * A23 * A34 * A45 * dA56dq6 * A67 * A78;
                    break;
                case 6:
                    term = A01 * dA12dq2 * A23 * A34 * A45 * A56 * dA67dq7 * A78;
                    break;
                }
            }
            else if (l == 2)
            {
                // Group 2: derivative in the third factor.
                switch (k)
                {
                case 0:
                    term = dA01dq1 * A12 * dA23dq3 * A34 * A45 * A56 * A67 * A78;
                    break;
                case 1:
                    term = A01 * dA12dq2 * dA23dq3 * A34 * A45 * A56 * A67 * A78;
                    break;
                case 2:
                    term = A01 * A12 * ddA23dq3dq3 * A34 * A45 * A56 * A67 * A78;
                    break;
                case 3:
                    term = A01 * A12 * dA23dq3 * dA34dq4 * A45 * A56 * A67 * A78;
                    break;
                case 4:
                    term = A01 * A12 * dA23dq3 * A34 * dA45dq5 * A56 * A67 * A78;
                    break;
                case 5:
                    term = A01 * A12 * dA23dq3 * A34 * A45 * dA56dq6 * A67 * A78;
                    break;
                case 6:
                    term = A01 * A12 * dA23dq3 * A34 * A45 * A56 * dA67dq7 * A78;
                    break;
                }
            }
            else if (l == 3)
            {
                // Group 3: derivative in the fourth factor.
                switch (k)
                {
                case 0:
                    term = dA01dq1 * A12 * A23 * dA34dq4 * A45 * A56 * A67 * A78;
                    break;
                case 1:
                    term = A01 * dA12dq2 * A23 * dA34dq4 * A45 * A56 * A67 * A78;
                    break;
                case 2:
                    term = A01 * A12 * dA23dq3 * dA34dq4 * A45 * A56 * A67 * A78;
                    break;
                case 3:
                    term = A01 * A12 * A23 * ddA34dq4dq4 * A45 * A56 * A67 * A78;
                    break;
                case 4:
                    term = A01 * A12 * A23 * dA34dq4 * dA45dq5 * A56 * A67 * A78;
                    break;
                case 5:
                    term = A01 * A12 * A23 * dA34dq4 * A45 * dA56dq6 * A67 * A78;
                    break;
                case 6:
                    term = A01 * A12 * A23 * dA34dq4 * A45 * A56 * dA67dq7 * A78;
                    break;
                }
            }
            else if (l == 4)
            {
                // Group 4: derivative in the fifth factor.
                switch (k)
                {
                case 0:
                    term = dA01dq1 * A12 * A23 * A34 * dA45dq5 * A56 * A67 * A78;
                    break;
                case 1:
                    term = A01 * dA12dq2 * A23 * A34 * dA45dq5 * A56 * A67 * A78;
                    break;
                case 2:
                    term = A01 * A12 * dA23dq3 * A34 * dA45dq5 * A56 * A67 * A78;
                    break;
                case 3:
                    term = A01 * A12 * A23 * dA34dq4 * dA45dq5 * A56 * A67 * A78;
                    break;
                case 4:
                    term = A01 * A12 * A23 * A34 * ddA45dq5dq5 * A56 * A67 * A78;
                    break;
                case 5:
                    term = A01 * A12 * A23 * A34 * dA45dq5 * dA56dq6 * A67 * A78;
                    break;
                case 6:
                    term = A01 * A12 * A23 * A34 * dA45dq5 * A56 * dA67dq7 * A78;
                    break;
                }
            }
            else if (l == 5)
            {
                // Group 5: derivative in the sixth factor.
                switch (k)
                {
                case 0:
                    term = dA01dq1 * A12 * A23 * A34 * A45 * dA56dq6 * A67 * A78;
                    break;
                case 1:
                    term = A01 * dA12dq2 * A23 * A34 * A45 * dA56dq6 * A67 * A78;
                    break;
                case 2:
                    term = A01 * A12 * dA23dq3 * A34 * A45 * dA56dq6 * A67 * A78;
                    break;
                case 3:
                    term = A01 * A12 * A23 * dA34dq4 * A45 * dA56dq6 * A67 * A78;
                    break;
                case 4:
                    term = A01 * A12 * A23 * A34 * dA45dq5 * dA56dq6 * A67 * A78;
                    break;
                case 5:
                    term = A01 * A12 * A23 * A34 * A45 * ddA56dq6dq6 * A67 * A78;
                    break;
                case 6:
                    term = A01 * A12 * A23 * A34 * A45 * dA56dq6 * dA67dq7 * A78;
                    break;
                }
            }
            else if (l == 6)
            {
                // Group 6: derivative in the seventh factor.
                switch (k)
                {
                case 0:
                    term = dA01dq1 * A12 * A23 * A34 * A45 * A56 * dA67dq7 * A78;
                    break;
                case 1:
                    term = A01 * dA12dq2 * A23 * A34 * A45 * A56 * dA67dq7 * A78;
                    break;
                case 2:
                    term = A01 * A12 * dA23dq3 * A34 * A45 * A56 * dA67dq7 * A78;
                    break;
                case 3:
                    term = A01 * A12 * A23 * dA34dq4 * A45 * A56 * dA67dq7 * A78;
                    break;
                case 4:
                    term = A01 * A12 * A23 * A34 * dA45dq5 * A56 * dA67dq7 * A78;
                    break;
                case 5:
                    term = A01 * A12 * A23 * A34 * A45 * dA56dq6 * dA67dq7 * A78;
                    break;
                case 6:
                    term = A01 * A12 * A23 * A34 * A45 * A56 * ddA67dq7dq7 * A78;
                    break;
                }
            }
            // Copy the 4x4 result into the tensor at slice (k, l).
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    ddA08dqdq(i, j, k, l) = term(i, j);
        }
    }
    Eigen::array<Eigen::Index, 4> offsets = {0, 3, 0, 0};              // start at row=0, col=3, third=0, fourth=0
    Eigen::array<Eigen::Index, 4> extents = {3, 1, 7, 7};              // 3 rows, 1 column, 7, 7
    Eigen::Tensor<double, 4> temp = ddA08dqdq.slice(offsets, extents); // shape (3,1,7,7)

    // Squeeze out the second dimension (size 1) manually into a rank-3 tensor of shape (3,7,7)
    Eigen::Tensor<double, 3> ddr800dqdq(3, 7, 7);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 7; ++j)
            for (int k = 0; k < 7; ++k)
                ddr800dqdq(i, j, k) = temp(i, 0, j, k);

    // --- Extract the rotation matrix from A08 and compute its elements ---
    Eigen::Matrix3d R08 = A08.block<3, 3>(0, 0);
    double r11 = R08(0, 0);
    double r21 = R08(1, 0);
    double r32 = R08(2, 1);
    double r33 = R08(2, 2);
    double r31 = -R08(2, 0);

    // --- Extract slices (squeezed) from ddA08dqdq for second derivatives ---
    Eigen::Matrix<double, 7, 7> ddyphidqdq, ddxphidqdq, ddythetadqdq, ddx1thetadqdq, ddx2thetadqdq, ddypsidqdq, ddxpsidqdq;
    for (int i = 0; i < 7; ++i)
    {
        for (int j = 0; j < 7; ++j)
        {
            ddyphidqdq(i, j) = ddA08dqdq(2, 1, i, j);

            ddxphidqdq(i, j) = ddA08dqdq(2, 2, i, j);

            ddythetadqdq(i, j) = ddA08dqdq(2, 0, i, j);

            ddx1thetadqdq(i, j) = ddA08dqdq(2, 1, i, j);

            ddx2thetadqdq(i, j) = ddA08dqdq(2, 2, i, j);

            ddypsidqdq(i, j) = ddA08dqdq(1, 0, i, j);

            ddxpsidqdq(i, j) = ddA08dqdq(0, 0, i, j);
        }
    }

    // --- Compute second derivative of phi ---
    Eigen::Matrix<double, 7, 7> ddphidqdq = -r32 * ddxphidqdq - (dyphidq * dxphidq.transpose()) + r33 * ddyphidqdq + (dxphidq * dyphidq.transpose());

    // --- Theta second derivative ---
    // Compute intermediate variables.
    double X = std::sqrt(r32 * r32 + r33 * r33);
    double Y = r31;
    double denom = X * X + Y * Y;

    // First derivative of X with respect to q:
    Eigen::Matrix<double, 7, 1> dX_dq = (r32 * dx1thetadq + r33 * dx2thetadq) / X;

    // Compute ddX_dqdq as a 7×7 matrix. For each row i and column j:
    Eigen::Matrix<double, 7, 7> ddX_dqdq;
    for (int i = 0; i < 7; ++i)
    {
        double term_vec = dx1thetadq(i) * dx1thetadq(i) + dx2thetadq(i) * dx2thetadq(i);
        double term_outer = std::pow(r32 * dx1thetadq(i) + r33 * dx2thetadq(i), 2) / (X * X);
        for (int j = 0; j < 7; ++j)
        {
            ddX_dqdq(i, j) = (r32 * ddx1thetadqdq(i, j) + r33 * ddx2thetadqdq(i, j) + term_vec - term_outer) / X;
        }
    }

    // For the quotient rule, build an outer term.
    // For each row i, compute t = (X*dythetadq(i) - Y*dX_dq(i))*(X*dX_dq(i) + Y*dythetadq(i))
    // and replicate it across columns.
    Eigen::Matrix<double, 7, 7> outerTerm;
    for (int i = 0; i < 7; ++i)
    {
        double t = (X * dythetadq(i) - Y * dX_dq(i)) * (X * dX_dq(i) + Y * dythetadq(i));
        for (int j = 0; j < 7; ++j)
        {
            outerTerm(i, j) = t;
        }
    }

    // Compute ddthetadqdq:
    Eigen::Matrix<double, 7, 7> ddthetadqdq = -(X * ddythetadqdq - Y * ddX_dqdq) / denom - 2 * outerTerm / (denom * denom);

    // --- Psi second derivative ---
    Eigen::Matrix<double, 7, 7> ddpsidqdq = -r21 * ddxpsidqdq - (dypsidq * dxpsidq.transpose()) + r11 * ddypsidqdq + (dxpsidq * dypsidq.transpose());

    // --- Assemble the analytical Hessian HA ---
    for (int k = 0; k < 7; ++k)
    {
        Eigen::Matrix<double, 6, 7> slice;
        for (int j = 0; j < 7; ++j)
        {
            for (int i = 0; i < 3; ++i)
            {
                slice(i, j) = ddr800dqdq(i, j, k);
            }
        }
        // Row 4 (index 3) from ddphidqdq: take its k-th column transposed
        slice.row(3) = ddphidqdq.col(k).transpose();
        // Row 5 (index 4) from ddthetadqdq: take its k-th column transposed
        slice.row(4) = ddthetadqdq.col(k).transpose();
        // Row 6 (index 5) from ddpsidqdq: take its k-th column transposed
        slice.row(5) = ddpsidqdq.col(k).transpose();

        HA[k] = slice;
    }
}

void SystemKG3FullInput::calcEffectorAnalyticalJacobian(const Eigen::VectorXd &q)
{
    // Update JA based on current q.
    // Derivative of matrix exponential of one-parameter subgroup
    Vec6d e6;
    e6 << 0, 0, 0, 0, 0, 1;

    dA01dq1 = kinematicsTranz(h1) * kinematicsRotx(pi) * kinematicsHatSE3(e6) * kinematicsRotz(q(0));
    dA12dq2 = kinematicsTranz(-h2) * kinematicsTrany(l2) * kinematicsRotx(pi2) * kinematicsHatSE3(e6) * kinematicsRotz(q(1));
    dA23dq3 = kinematicsTrany(-h3) * kinematicsTranz(-l3) * kinematicsRotx(-pi2) * kinematicsHatSE3(e6) * kinematicsRotz(q(2));
    dA34dq4 = kinematicsTranz(-h4) * kinematicsTrany(l4) * kinematicsRotx(pi2) * kinematicsHatSE3(e6) * kinematicsRotz(q(3));
    dA45dq5 = kinematicsTrany(-h5) * kinematicsTranz(-l5) * kinematicsRotx(-pi2) * kinematicsHatSE3(e6) * kinematicsRotz(q(4));
    dA56dq6 = kinematicsTranz(-h6) * kinematicsRotx(pi2) * kinematicsHatSE3(e6) * kinematicsRotz(q(5));
    dA67dq7 = kinematicsTrany(-h7) * kinematicsRotx(-pi2) * kinematicsHatSE3(e6) * kinematicsRotz(q(6));

    Eigen::Matrix4d dA08dq1 = dA01dq1 * A12 * A23 * A34 * A45 * A56 * A67 * A78;
    Eigen::Matrix4d dA08dq2 = A01 * dA12dq2 * A23 * A34 * A45 * A56 * A67 * A78;
    Eigen::Matrix4d dA08dq3 = A01 * A12 * dA23dq3 * A34 * A45 * A56 * A67 * A78;
    Eigen::Matrix4d dA08dq4 = A01 * A12 * A23 * dA34dq4 * A45 * A56 * A67 * A78;
    Eigen::Matrix4d dA08dq5 = A01 * A12 * A23 * A34 * dA45dq5 * A56 * A67 * A78;
    Eigen::Matrix4d dA08dq6 = A01 * A12 * A23 * A34 * A45 * dA56dq6 * A67 * A78;
    Eigen::Matrix4d dA08dq7 = A01 * A12 * A23 * A34 * A45 * A56 * dA67dq7 * A78;

    Eigen::Matrix<double, 3, 7> dr800dq;
    dr800dq.col(0) = dA08dq1.block(0, 3, 3, 1);
    dr800dq.col(1) = dA08dq2.block(0, 3, 3, 1);
    dr800dq.col(2) = dA08dq3.block(0, 3, 3, 1);
    dr800dq.col(3) = dA08dq4.block(0, 3, 3, 1);
    dr800dq.col(4) = dA08dq5.block(0, 3, 3, 1);
    dr800dq.col(5) = dA08dq6.block(0, 3, 3, 1);
    dr800dq.col(6) = dA08dq7.block(0, 3, 3, 1);

    Eigen::Matrix3d R08 = A08.block<3, 3>(0, 0); // Rotation matrix

    double r11 = R08(0, 0);
    double r21 = R08(1, 0);
    double r32 = R08(2, 1);
    double r33 = R08(2, 2);
    double r31 = -R08(2, 0);

    // Theta calculations
    double x_val = std::sqrt(r32 * r32 + r33 * r33);
    double dthetady = x_val / (x_val * x_val + r31 * r31);
    double dthetadr32 = (-r31 / (x_val * x_val + r31 * r31)) * (r32 / x_val);
    double dthetadr33 = (-r31 / (x_val * x_val + r31 * r31)) * (r33 / x_val);

    dythetadq << -dA08dq1(2, 0), -dA08dq2(2, 0), -dA08dq3(2, 0),
        -dA08dq4(2, 0), -dA08dq5(2, 0), -dA08dq6(2, 0), -dA08dq7(2, 0);

    dx1thetadq << dA08dq1(2, 1), dA08dq2(2, 1), dA08dq3(2, 1),
        dA08dq4(2, 1), dA08dq5(2, 1), dA08dq6(2, 1), dA08dq7(2, 1);

    dx2thetadq << dA08dq1(2, 2), dA08dq2(2, 2), dA08dq3(2, 2),
        dA08dq4(2, 2), dA08dq5(2, 2), dA08dq6(2, 2), dA08dq7(2, 2);

    Eigen::Matrix<double, 7, 1> dthetadq = dthetady * dythetadq + dthetadr32 * dx1thetadq + dthetadr33 * dx2thetadq;

    // Phi calculations
    double dphidy = r33 / (r33 * r33 + r32 * r32);
    double dphidx = -r32 / (r33 * r33 + r32 * r32);

    dxphidq = dx2thetadq;
    dyphidq = dx1thetadq;
    // For phi, dyphidq = dx1thetadq and dxphidq = dx2thetadq.
    Eigen::Matrix<double, 7, 1> dphidq = dphidy * dx1thetadq + dphidx * dx2thetadq;

    // Psi calculations
    double dpsidy = r11 / (r11 * r11 + r21 * r21);
    double dpsidx = -r21 / (r11 * r11 + r21 * r21);

    dypsidq << dA08dq1(1, 0), dA08dq2(1, 0), dA08dq3(1, 0),
        dA08dq4(1, 0), dA08dq5(1, 0), dA08dq6(1, 0), dA08dq7(1, 0);

    dxpsidq << dA08dq1(0, 0), dA08dq2(0, 0), dA08dq3(0, 0),
        dA08dq4(0, 0), dA08dq5(0, 0), dA08dq6(0, 0), dA08dq7(0, 0);

    Eigen::Matrix<double, 7, 1> dpsidq = dpsidy * dypsidq + dpsidx * dxpsidq;

    JA.block<3, 7>(0, 0) = dr800dq;
    JA.block<1, 7>(3, 0) = dphidq.transpose();
    JA.block<1, 7>(4, 0) = dthetadq.transpose();
    JA.block<1, 7>(5, 0) = dpsidq.transpose();
}

void SystemKG3FullInput::calcEffectorFKM(const Eigen::VectorXd &q)
{
    // Update xe based on current q.
    // Use the provided tranx, trany, tranz, rotx, roty, rotz functions to build the homogeneous transformations below
    A01 = kinematicsTranz(h1) * kinematicsRotx(pi) * kinematicsRotz(q(0));
    A12 = kinematicsTranz(-h2) * kinematicsTrany(l2) * kinematicsRotx(pi2) * kinematicsRotz(q(1));
    A23 = kinematicsTrany(-h3) * kinematicsTranz(-l3) * kinematicsRotx(-pi2) * kinematicsRotz(q(2));
    A34 = kinematicsTranz(-h4) * kinematicsTrany(l4) * kinematicsRotx(pi2) * kinematicsRotz(q(3));
    A45 = kinematicsTrany(-h5) * kinematicsTranz(-l5) * kinematicsRotx(-pi2) * kinematicsRotz(q(4));
    A56 = kinematicsTranz(-h6) * kinematicsRotx(pi2) * kinematicsRotz(q(5));
    A67 = kinematicsTrany(-h7) * kinematicsRotx(-pi2) * kinematicsRotz(q(6));
    A78 = kinematicsTranz(-h8) * kinematicsRotx(pi);

    A08 = A01 * A12 * A23 * A34 * A45 * A56 * A67 * A78;

    A02 = A01 * A12;
    A03 = A02 * A23;
    A04 = A03 * A34;
    A05 = A04 * A45;
    A06 = A05 * A56;
    A07 = A06 * A67;
    A08 = A07 * A78;

    R01 = A01.block<3, 3>(0, 0); // Rotation matrix from base to link 1
    R02 = A02.block<3, 3>(0, 0); // Rotation matrix from base to link 2
    R03 = A03.block<3, 3>(0, 0); // Rotation matrix from base to link 3
    R04 = A04.block<3, 3>(0, 0); // Rotation matrix from base to link 4
    R05 = A05.block<3, 3>(0, 0); // Rotation matrix from base to link 5
    R06 = A06.block<3, 3>(0, 0); // Rotation matrix from base to link 6
    R07 = A07.block<3, 3>(0, 0); // Rotation matrix from base to link 7

    // Rotation matrix and translation vector
    R08 = A08.block<3, 3>(0, 0);                  // Rotation matrix from base to end effector
    Eigen::Vector3d r800 = A08.block<3, 1>(0, 3); // End effector position

    // End effector state = [x,y,z,phi,theta,psi]
    yphi = R08(2, 1);
    xphi = R08(2, 2);
    const double phi = atan2(yphi, xphi);

    ytheta = -R08(2, 0);
    xtheta = sqrt((R08(2, 1) * R08(2, 1)) + (R08(2, 2) * R08(2, 2)));
    const double theta = atan2(ytheta, xtheta);

    ypsi = R08(1, 0);
    xpsi = R08(0, 0);
    const double psi = atan2(ypsi, xpsi);

    xe << r800(0), r800(1), r800(2), phi, theta, psi; // End effector state
}

void SystemKG3FullInput::calcLinkGeometricJacobian(const Eigen::VectorXd &q)
{

    // Unit vectors for the axes of rotation
    Eigen::Vector3d z00;
    z00 << 0, 0, 1; // Base frame Z-axis
    Eigen::Vector3d z10 = A01.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z20 = A02.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z30 = A03.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z40 = A04.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z50 = A05.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z60 = A06.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z70 = A07.block<3, 3>(0, 0) * z00;

    // Translation vectors from each coordinate frame to the base frame
    Eigen::Vector3d r100 = A01.block<3, 1>(0, 3);
    Eigen::Vector3d r200 = A02.block<3, 1>(0, 3);
    Eigen::Vector3d r300 = A03.block<3, 1>(0, 3);
    Eigen::Vector3d r400 = A04.block<3, 1>(0, 3);
    Eigen::Vector3d r500 = A05.block<3, 1>(0, 3);
    Eigen::Vector3d r600 = A06.block<3, 1>(0, 3);
    Eigen::Vector3d r700 = A07.block<3, 1>(0, 3);

    // Homog transformation matrices from previous frame to next centre of mass of each link
    A1c2 = kinematicsRotz(q(0)) * kinematicsTranx(xc2) * kinematicsTrany(yc2) * kinematicsTranz(zc2);
    A2c3 = kinematicsRotz(q(1)) * kinematicsTranx(xc3) * kinematicsTrany(yc3) * kinematicsTranz(zc3);
    A3c4 = kinematicsRotz(q(2)) * kinematicsTranx(xc4) * kinematicsTrany(yc4) * kinematicsTranz(zc4);
    A4c5 = kinematicsRotz(q(3)) * kinematicsTranx(xc5) * kinematicsTrany(yc5) * kinematicsTranz(zc5);
    A5c6 = kinematicsRotz(q(4)) * kinematicsTranx(xc6) * kinematicsTrany(yc6) * kinematicsTranz(zc6);
    A6c7 = kinematicsRotz(q(5)) * kinematicsTranx(xc7) * kinematicsTrany(yc7) * kinematicsTranz(zc7);
    A7c8 = kinematicsRotz(q(6)) * kinematicsTranx(xc8) * kinematicsTrany(yc8) * kinematicsTranz(zc8);

    // Compute the transformation matrices from the base frame to the center of mass of each link
    A0c2 = A01 * A1c2;
    A0c3 = A02 * A2c3;
    A0c4 = A03 * A3c4;
    A0c5 = A04 * A4c5;
    A0c6 = A05 * A5c6;
    A0c7 = A06 * A6c7;
    A0c8 = A07 * A7c8;

    // Translation vectors from base frame to centre of mass for each link
    Eigen::Vector3d rc200 = A0c2.block<3, 1>(0, 3);
    Eigen::Vector3d rc300 = A0c3.block<3, 1>(0, 3);
    Eigen::Vector3d rc400 = A0c4.block<3, 1>(0, 3);
    Eigen::Vector3d rc500 = A0c5.block<3, 1>(0, 3);
    Eigen::Vector3d rc600 = A0c6.block<3, 1>(0, 3);
    Eigen::Vector3d rc700 = A0c7.block<3, 1>(0, 3);
    Eigen::Vector3d rc800 = A0c8.block<3, 1>(0, 3);

    // Decalre the 6x1 column vectors.
    Eigen::Matrix<double, 6, 1> jc22, jc32, jc33, jc42, jc43, jc44;
    Eigen::Matrix<double, 6, 1> jc52, jc53, jc54, jc55, jc62, jc63, jc64, jc65, jc66;
    Eigen::Matrix<double, 6, 1> jc72, jc73, jc74, jc75, jc76, jc77, jc82, jc83, jc84, jc85, jc86, jc87, jc88;

    // Example: jc22 = [skew(z10)*(rc200 - r100); z10];
    jc22.topRows(3) = kinematicsSkew(z10) * (rc200 - r100);
    jc22.bottomRows(3) = z10;

    // jc32 = [skew(z10)*(rc300 - r100); z10];
    jc32.topRows(3) = kinematicsSkew(z10) * (rc300 - r100);
    jc32.bottomRows(3) = z10;

    // jc33 = [skew(z20)*(rc300 - r200); z20];
    jc33.topRows(3) = kinematicsSkew(z20) * (rc300 - r200);
    jc33.bottomRows(3) = z20;

    // jc42 = [skew(z10)*(rc400 - r100); z10];
    jc42.topRows(3) = kinematicsSkew(z10) * (rc400 - r100);
    jc42.bottomRows(3) = z10;

    // jc43 = [skew(z20)*(rc400 - r200); z20];
    jc43.topRows(3) = kinematicsSkew(z20) * (rc400 - r200);
    jc43.bottomRows(3) = z20;

    // jc44 = [skew(z30)*(rc400 - r300); z30];
    jc44.topRows(3) = kinematicsSkew(z30) * (rc400 - r300);
    jc44.bottomRows(3) = z30;

    // jc52 = [skew(z10)*(rc500 - r100); z10];
    jc52.topRows(3) = kinematicsSkew(z10) * (rc500 - r100);
    jc52.bottomRows(3) = z10;

    // jc53 = [skew(z20)*(rc500 - r200); z20];
    jc53.topRows(3) = kinematicsSkew(z20) * (rc500 - r200);
    jc53.bottomRows(3) = z20;

    // jc54 = [skew(z30)*(rc500 - r300); z30];
    jc54.topRows(3) = kinematicsSkew(z30) * (rc500 - r300);
    jc54.bottomRows(3) = z30;

    // jc55 = [skew(z40)*(rc500 - r400); z40];
    jc55.topRows(3) = kinematicsSkew(z40) * (rc500 - r400);
    jc55.bottomRows(3) = z40;

    // jc62 = [skew(z10)*(rc600 - r100); z10];
    jc62.topRows(3) = kinematicsSkew(z10) * (rc600 - r100);
    jc62.bottomRows(3) = z10;

    // jc63 = [skew(z20)*(rc600 - r200); z20];
    jc63.topRows(3) = kinematicsSkew(z20) * (rc600 - r200);
    jc63.bottomRows(3) = z20;

    // jc64 = [skew(z30)*(rc600 - r300); z30];
    jc64.topRows(3) = kinematicsSkew(z30) * (rc600 - r300);
    jc64.bottomRows(3) = z30;

    // jc65 = [skew(z40)*(rc600 - r400); z40];
    jc65.topRows(3) = kinematicsSkew(z40) * (rc600 - r400);
    jc65.bottomRows(3) = z40;

    // jc66 = [skew(z50)*(rc600 - r500); z50];
    jc66.topRows(3) = kinematicsSkew(z50) * (rc600 - r500);
    jc66.bottomRows(3) = z50;

    // jc72 = [skew(z10)*(rc700 - r100); z10];
    jc72.topRows(3) = kinematicsSkew(z10) * (rc700 - r100);
    jc72.bottomRows(3) = z10;

    // jc73 = [skew(z20)*(rc700 - r200); z20];
    jc73.topRows(3) = kinematicsSkew(z20) * (rc700 - r200);
    jc73.bottomRows(3) = z20;

    // jc74 = [skew(z30)*(rc700 - r300); z30];
    jc74.topRows(3) = kinematicsSkew(z30) * (rc700 - r300);
    jc74.bottomRows(3) = z30;

    // jc75 = [skew(z40)*(rc700 - r400); z40];
    jc75.topRows(3) = kinematicsSkew(z40) * (rc700 - r400);
    jc75.bottomRows(3) = z40;

    // jc76 = [skew(z50)*(rc700 - r500); z50];
    jc76.topRows(3) = kinematicsSkew(z50) * (rc700 - r500);
    jc76.bottomRows(3) = z50;

    // jc77 = [skew(z60)*(rc700 - r600); z60];
    jc77.topRows(3) = kinematicsSkew(z60) * (rc700 - r600);
    jc77.bottomRows(3) = z60;

    // jc82 = [skew(z10)*(rc800 - r100); z10];
    jc82.topRows(3) = kinematicsSkew(z10) * (rc800 - r100);
    jc82.bottomRows(3) = z10;

    // jc83 = [skew(z20)*(rc800 - r200); z20];
    jc83.topRows(3) = kinematicsSkew(z20) * (rc800 - r200);
    jc83.bottomRows(3) = z20;

    // jc84 = [skew(z30)*(rc800 - r300); z30];
    jc84.topRows(3) = kinematicsSkew(z30) * (rc800 - r300);
    jc84.bottomRows(3) = z30;

    // jc85 = [skew(z40)*(rc800 - r400); z40];
    jc85.topRows(3) = kinematicsSkew(z40) * (rc800 - r400);
    jc85.bottomRows(3) = z40;

    // jc86 = [skew(z50)*(rc800 - r500); z50];
    jc86.topRows(3) = kinematicsSkew(z50) * (rc800 - r500);
    jc86.bottomRows(3) = z50;

    // jc87 = [skew(z60)*(rc800 - r600); z60];
    jc87.topRows(3) = kinematicsSkew(z60) * (rc800 - r600);
    jc87.bottomRows(3) = z60;

    // jc88 = [skew(z70)*(rc800 - r700); z70];
    jc88.topRows(3) = kinematicsSkew(z70) * (rc800 - r700);
    jc88.bottomRows(3) = z70;

    // Now form the overall Jacobians as 6x7 matrices.
    Jc1.setZero();

    // Jc2 = [jc22, zeros, ..., zeros]
    Jc2.setZero();
    Jc2.col(0) = jc22;

    // Jc3 = [jc32, jc33, zeros, ..., zeros]
    Jc3.setZero();
    Jc3.col(0) = jc32;
    Jc3.col(1) = jc33;

    // Jc4 = [jc42, jc43, jc44, zeros, zeros, zeros, zeros]
    Jc4.setZero();
    Jc4.col(0) = jc42;
    Jc4.col(1) = jc43;
    Jc4.col(2) = jc44;

    // Jc5 = [jc52, jc53, jc54, jc55, zeros, zeros, zeros]
    Jc5.setZero();
    Jc5.col(0) = jc52;
    Jc5.col(1) = jc53;
    Jc5.col(2) = jc54;
    Jc5.col(3) = jc55;

    // Jc6 = [jc62, jc63, jc64, jc65, jc66, zeros, zeros]
    Jc6.setZero();
    Jc6.col(0) = jc62;
    Jc6.col(1) = jc63;
    Jc6.col(2) = jc64;
    Jc6.col(3) = jc65;
    Jc6.col(4) = jc66;

    // Jc7 = [jc72, jc73, jc74, jc75, jc76, jc77, zeros]
    Jc7.setZero();
    Jc7.col(0) = jc72;
    Jc7.col(1) = jc73;
    Jc7.col(2) = jc74;
    Jc7.col(3) = jc75;
    Jc7.col(4) = jc76;
    Jc7.col(5) = jc77;

    // Jc8 = [jc82, jc83, jc84, jc85, jc86, jc87, jc88]
    Jc8.setZero();
    Jc8.col(0) = jc82;
    Jc8.col(1) = jc83;
    Jc8.col(2) = jc84;
    Jc8.col(3) = jc85;
    Jc8.col(4) = jc86;
    Jc8.col(5) = jc87;
    Jc8.col(6) = jc88;
}

void SystemKG3FullInput::calcLinkGeometricHessian(const Eigen::VectorXd &q)
{
    // Update HA based on current q.

    // Unit vectors for the axes of rotation
    Eigen::Vector3d z00;
    z00 << 0, 0, 1; // Base frame Z-axis
    Eigen::Vector3d z10 = A01.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z20 = A02.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z30 = A03.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z40 = A04.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z50 = A05.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z60 = A06.block<3, 3>(0, 0) * z00;
    Eigen::Vector3d z70 = A07.block<3, 3>(0, 0) * z00;

    // Translation vectors from each coordinate frame to the base frame
    Eigen::Vector3d r100 = A01.block<3, 1>(0, 3);
    Eigen::Vector3d r200 = A02.block<3, 1>(0, 3);
    Eigen::Vector3d r300 = A03.block<3, 1>(0, 3);
    Eigen::Vector3d r400 = A04.block<3, 1>(0, 3);
    Eigen::Vector3d r500 = A05.block<3, 1>(0, 3);
    Eigen::Vector3d r600 = A06.block<3, 1>(0, 3);
    Eigen::Vector3d r700 = A07.block<3, 1>(0, 3);

    // Translation vectors from base frame to centre of mass for each link
    Eigen::Vector3d rc200 = A0c2.block<3, 1>(0, 3);
    Eigen::Vector3d rc300 = A0c3.block<3, 1>(0, 3);
    Eigen::Vector3d rc400 = A0c4.block<3, 1>(0, 3);
    Eigen::Vector3d rc500 = A0c5.block<3, 1>(0, 3);
    Eigen::Vector3d rc600 = A0c6.block<3, 1>(0, 3);
    Eigen::Vector3d rc700 = A0c7.block<3, 1>(0, 3);
    Eigen::Vector3d rc800 = A0c8.block<3, 1>(0, 3);

    // dA01dq = cat(3, dA01dq1, zeros, zeros, zeros, zeros, zeros, zeros)
    setTensorSlice(dA01dq, 0, dA01dq1);
    // Other slices are already zero

    // dA02dq = cat(3, dA01dq1*A12, A01*dA12dq2, zeros, zeros, zeros, zeros, zeros)
    setTensorSlice(dA02dq, 0, dA01dq1 * A12);
    setTensorSlice(dA02dq, 1, A01 * dA12dq2);

    // dA03dq = cat(3, dA01dq1*A12*A23, A01*dA12dq2*A23, A01*A12*dA23dq3, zeros, zeros, zeros, zeros)
    setTensorSlice(dA03dq, 0, dA01dq1 * A12 * A23);
    setTensorSlice(dA03dq, 1, A01 * dA12dq2 * A23);
    setTensorSlice(dA03dq, 2, A01 * A12 * dA23dq3);

    // dA04dq = cat(3, dA01dq1*A12*A23*A34, ...)
    setTensorSlice(dA04dq, 0, dA01dq1 * A12 * A23 * A34);
    setTensorSlice(dA04dq, 1, A01 * dA12dq2 * A23 * A34);
    setTensorSlice(dA04dq, 2, A01 * A12 * dA23dq3 * A34);
    setTensorSlice(dA04dq, 3, A01 * A12 * A23 * dA34dq4);

    // dA05dq
    setTensorSlice(dA05dq, 0, dA01dq1 * A12 * A23 * A34 * A45);
    setTensorSlice(dA05dq, 1, A01 * dA12dq2 * A23 * A34 * A45);
    setTensorSlice(dA05dq, 2, A01 * A12 * dA23dq3 * A34 * A45);
    setTensorSlice(dA05dq, 3, A01 * A12 * A23 * dA34dq4 * A45);
    setTensorSlice(dA05dq, 4, A01 * A12 * A23 * A34 * dA45dq5);

    // dA06dq
    setTensorSlice(dA06dq, 0, dA01dq1 * A12 * A23 * A34 * A45 * A56);
    setTensorSlice(dA06dq, 1, A01 * dA12dq2 * A23 * A34 * A45 * A56);
    setTensorSlice(dA06dq, 2, A01 * A12 * dA23dq3 * A34 * A45 * A56);
    setTensorSlice(dA06dq, 3, A01 * A12 * A23 * dA34dq4 * A45 * A56);
    setTensorSlice(dA06dq, 4, A01 * A12 * A23 * A34 * dA45dq5 * A56);
    setTensorSlice(dA06dq, 5, A01 * A12 * A23 * A34 * A45 * dA56dq6);

    // dA07dq
    setTensorSlice(dA07dq, 0, dA01dq1 * A12 * A23 * A34 * A45 * A56 * A67);
    setTensorSlice(dA07dq, 1, A01 * dA12dq2 * A23 * A34 * A45 * A56 * A67);
    setTensorSlice(dA07dq, 2, A01 * A12 * dA23dq3 * A34 * A45 * A56 * A67);
    setTensorSlice(dA07dq, 3, A01 * A12 * A23 * dA34dq4 * A45 * A56 * A67);
    setTensorSlice(dA07dq, 4, A01 * A12 * A23 * A34 * dA45dq5 * A56 * A67);
    setTensorSlice(dA07dq, 5, A01 * A12 * A23 * A34 * A45 * dA56dq6 * A67);
    setTensorSlice(dA07dq, 6, A01 * A12 * A23 * A34 * A45 * A56 * dA67dq7);

    // dA08dq
    setTensorSlice(dA08dq, 0, dA01dq1 * A12 * A23 * A34 * A45 * A56 * A67 * A78);
    setTensorSlice(dA08dq, 1, A01 * dA12dq2 * A23 * A34 * A45 * A56 * A67 * A78);
    setTensorSlice(dA08dq, 2, A01 * A12 * dA23dq3 * A34 * A45 * A56 * A67 * A78);
    setTensorSlice(dA08dq, 3, A01 * A12 * A23 * dA34dq4 * A45 * A56 * A67 * A78);
    setTensorSlice(dA08dq, 4, A01 * A12 * A23 * A34 * dA45dq5 * A56 * A67 * A78);
    setTensorSlice(dA08dq, 5, A01 * A12 * A23 * A34 * A45 * dA56dq6 * A67 * A78);
    setTensorSlice(dA08dq, 6, A01 * A12 * A23 * A34 * A45 * A56 * dA67dq7 * A78);

    // --- Extract translation derivatives from dA?dq tensors ---
    Eigen::Matrix<double, 3, 7> dr100dq, dr200dq, dr300dq, dr400dq, dr500dq, dr600dq, dr700dq;
    for (int k = 0; k < 7; ++k)
    {
        for (int i = 0; i < 3; ++i)
        {
            dr100dq(i, k) = dA01dq(i, 3, k);
            dr200dq(i, k) = dA02dq(i, 3, k);
            dr300dq(i, k) = dA03dq(i, 3, k);
            dr400dq(i, k) = dA04dq(i, 3, k);
            dr500dq(i, k) = dA05dq(i, 3, k);
            dr600dq(i, k) = dA06dq(i, 3, k);
            dr700dq(i, k) = dA07dq(i, 3, k);
        }
    }

    // --- Derivatives of COM from base frame with respect to q ---
    Vec6d e6;
    e6 << 0, 0, 0, 0, 0, 1;
    Eigen::Matrix4d dA1c2dq1 = kinematicsHatSE3(e6) * A1c2;
    Eigen::Matrix4d dA2c3dq2 = kinematicsHatSE3(e6) * A2c3;
    Eigen::Matrix4d dA3c4dq3 = kinematicsHatSE3(e6) * A3c4;
    Eigen::Matrix4d dA4c5dq4 = kinematicsHatSE3(e6) * A4c5;
    Eigen::Matrix4d dA5c6dq5 = kinematicsHatSE3(e6) * A5c6;
    Eigen::Matrix4d dA6c7dq6 = kinematicsHatSE3(e6) * A6c7;
    Eigen::Matrix4d dA7c8dq7 = kinematicsHatSE3(e6) * A7c8;

    // --- Derivatives of COM transformations with respect to q ---

    // A0c2
    Eigen::Matrix4d dA0c2dq1 = dA01dq1 * A1c2 + A01 * dA1c2dq1;
    setTensorSlice(dA0c2dq, 0, dA0c2dq1);
    // Other slices are already zero

    // A0c3
    Eigen::Matrix4d dA02dq2 = A01 * dA12dq2;
    Eigen::Matrix4d dA0c3dq1 = (dA01dq1 * A12) * A2c3;
    Eigen::Matrix4d dA0c3dq2 = dA02dq2 * A2c3 + A02 * dA2c3dq2;
    setTensorSlice(dA0c3dq, 0, dA0c3dq1);
    setTensorSlice(dA0c3dq, 1, dA0c3dq2);
    // Other slices are already zero

    // A0c4
    Eigen::Matrix4d dA02dq1 = dA01dq1 * A12;
    Eigen::Matrix4d dA03dq1 = dA02dq1 * A23;
    Eigen::Matrix4d dA03dq2 = dA02dq2 * A23;
    Eigen::Matrix4d dA03dq3 = A02 * dA23dq3;
    setTensorSlice(dA0c4dq, 0, dA03dq1 * A3c4);
    setTensorSlice(dA0c4dq, 1, dA03dq2 * A3c4);
    setTensorSlice(dA0c4dq, 2, (dA03dq3 * A3c4) + (A03 * dA3c4dq3));
    // Other slices are already zero

    // A0c5
    Eigen::Matrix4d dA04dq1 = dA03dq1 * A34;
    Eigen::Matrix4d dA04dq2 = dA03dq2 * A34;
    Eigen::Matrix4d dA04dq3 = dA03dq3 * A34;
    Eigen::Matrix4d dA04dq4 = A03 * dA34dq4;
    setTensorSlice(dA0c5dq, 0, dA04dq1 * A4c5);
    setTensorSlice(dA0c5dq, 1, dA04dq2 * A4c5);
    setTensorSlice(dA0c5dq, 2, dA04dq3 * A4c5);
    setTensorSlice(dA0c5dq, 3, (A03 * dA34dq4 * A4c5) + (A03 * A34 * dA4c5dq4));
    // Other slices are already zero

    // A0c6
    Eigen::Matrix4d dA05dq1 = dA04dq1 * A45;
    Eigen::Matrix4d dA05dq2 = dA04dq2 * A45;
    Eigen::Matrix4d dA05dq3 = dA04dq3 * A45;
    Eigen::Matrix4d dA05dq4 = dA04dq4 * A45;
    Eigen::Matrix4d dA05dq5 = A04 * dA45dq5;
    setTensorSlice(dA0c6dq, 0, dA05dq1 * A5c6);
    setTensorSlice(dA0c6dq, 1, dA05dq2 * A5c6);
    setTensorSlice(dA0c6dq, 2, dA05dq3 * A5c6);
    setTensorSlice(dA0c6dq, 3, dA05dq4 * A5c6);
    setTensorSlice(dA0c6dq, 4, (dA05dq5 * A5c6) + (A05 * dA5c6dq5));
    // Other slices are already zero

    // A0c7
    Eigen::Matrix4d dA06dq1 = dA05dq1 * A56;
    Eigen::Matrix4d dA06dq2 = dA05dq2 * A56;
    Eigen::Matrix4d dA06dq3 = dA05dq3 * A56;
    Eigen::Matrix4d dA06dq4 = dA05dq4 * A56;
    Eigen::Matrix4d dA06dq5 = dA05dq5 * A56;
    Eigen::Matrix4d dA06dq6 = A05 * dA56dq6;
    setTensorSlice(dA0c7dq, 0, dA06dq1 * A6c7);
    setTensorSlice(dA0c7dq, 1, dA06dq2 * A6c7);
    setTensorSlice(dA0c7dq, 2, dA06dq3 * A6c7);
    setTensorSlice(dA0c7dq, 3, dA06dq4 * A6c7);
    setTensorSlice(dA0c7dq, 4, dA06dq5 * A6c7);
    setTensorSlice(dA0c7dq, 5, (A05 * A56 * dA6c7dq6) + ((A05 * dA56dq6) * A6c7));
    // Other slices are already zero

    // A0c8
    Eigen::Matrix4d dA07dq1 = dA06dq1 * A67;
    Eigen::Matrix4d dA07dq2 = dA06dq2 * A67;
    Eigen::Matrix4d dA07dq3 = dA06dq3 * A67;
    Eigen::Matrix4d dA07dq4 = dA06dq4 * A67;
    Eigen::Matrix4d dA07dq5 = dA06dq5 * A67;
    Eigen::Matrix4d dA07dq6 = dA06dq6 * A67;
    setTensorSlice(dA0c8dq, 0, dA07dq1 * A7c8);
    setTensorSlice(dA0c8dq, 1, dA07dq2 * A7c8);
    setTensorSlice(dA0c8dq, 2, dA07dq3 * A7c8);
    setTensorSlice(dA0c8dq, 3, dA07dq4 * A7c8);
    setTensorSlice(dA0c8dq, 4, dA07dq5 * A7c8);
    setTensorSlice(dA0c8dq, 5, dA07dq6 * A7c8);
    setTensorSlice(dA0c8dq, 6, (A06 * A67 * dA7c8dq7) + ((A06 * dA67dq7) * A7c8));

    // Derivatives of COM trans vectors w.r.t q
    Eigen::Matrix<double, 3, 7> drc200dq, drc300dq, drc400dq, drc500dq, drc600dq, drc700dq, drc800dq;
    for (int k = 0; k < 7; ++k)
    {
        for (int i = 0; i < 3; ++i)
        {
            drc200dq(i, k) = dA0c2dq(i, 3, k);
            drc300dq(i, k) = dA0c3dq(i, 3, k);
            drc400dq(i, k) = dA0c4dq(i, 3, k);
            drc500dq(i, k) = dA0c5dq(i, 3, k);
            drc600dq(i, k) = dA0c6dq(i, 3, k);
            drc700dq(i, k) = dA0c7dq(i, 3, k);
            drc800dq(i, k) = dA0c8dq(i, 3, k);
        }
    }

    // Derivatives of basis vectors w.r.t q
    Eigen::Matrix<double, 3, 7> dz10dq, dz20dq, dz30dq, dz40dq, dz50dq, dz60dq, dz70dq;
    for (int k = 0; k < 7; ++k)
    {
        for (int i = 0; i < 3; ++i)
        {
            dz10dq(i, k) = dA01dq(i, 2, k);
            dz20dq(i, k) = dA02dq(i, 2, k);
            dz30dq(i, k) = dA03dq(i, 2, k);
            dz40dq(i, k) = dA04dq(i, 2, k);
            dz50dq(i, k) = dA05dq(i, 2, k);
            dz60dq(i, k) = dA06dq(i, 2, k);
            dz70dq(i, k) = dA07dq(i, 2, k);
        }
    }

    // Main loop for computing Jacobian derivatives
    for (int i = 0; i < 7; ++i)
    {
        // dJc1dq - all zeros
        Eigen::Matrix<double, 6, 7> slice = Eigen::Matrix<double, 6, 7>::Zero();
        setJacobianTensorSlice(dJc1dq, i, slice);

        // dJc2dq
        slice.setZero();
        setJacobianColumn(slice, 0,
                          kinematicsSkew(dz10dq.col(i)) * (rc200 - r100) + kinematicsSkew(z10) * (drc200dq.col(i) - dr100dq.col(i)),
                          dz10dq.col(i));
        setJacobianTensorSlice(dJc2dq, i, slice);

        // dJc3dq
        slice.setZero();
        setJacobianColumn(slice, 0,
                          kinematicsSkew(dz10dq.col(i)) * (rc300 - r100) + kinematicsSkew(z10) * (drc300dq.col(i) - dr100dq.col(i)),
                          dz10dq.col(i));
        setJacobianColumn(slice, 1,
                          kinematicsSkew(dz20dq.col(i)) * (rc300 - r200) + kinematicsSkew(z20) * (drc300dq.col(i) - dr200dq.col(i)),
                          dz20dq.col(i));
        setJacobianTensorSlice(dJc3dq, i, slice);

        // dJc4dq
        slice.setZero();
        setJacobianColumn(slice, 0,
                          kinematicsSkew(dz10dq.col(i)) * (rc400 - r100) + kinematicsSkew(z10) * (drc400dq.col(i) - dr100dq.col(i)),
                          dz10dq.col(i));
        setJacobianColumn(slice, 1,
                          kinematicsSkew(dz20dq.col(i)) * (rc400 - r200) + kinematicsSkew(z20) * (drc400dq.col(i) - dr200dq.col(i)),
                          dz20dq.col(i));
        setJacobianColumn(slice, 2,
                          kinematicsSkew(dz30dq.col(i)) * (rc400 - r300) + kinematicsSkew(z30) * (drc400dq.col(i) - dr300dq.col(i)),
                          dz30dq.col(i));
        setJacobianTensorSlice(dJc4dq, i, slice);

        // dJc5dq
        slice.setZero();
        setJacobianColumn(slice, 0,
                          kinematicsSkew(dz10dq.col(i)) * (rc500 - r100) + kinematicsSkew(z10) * (drc500dq.col(i) - dr100dq.col(i)),
                          dz10dq.col(i));
        setJacobianColumn(slice, 1,
                          kinematicsSkew(dz20dq.col(i)) * (rc500 - r200) + kinematicsSkew(z20) * (drc500dq.col(i) - dr200dq.col(i)),
                          dz20dq.col(i));
        setJacobianColumn(slice, 2,
                          kinematicsSkew(dz30dq.col(i)) * (rc500 - r300) + kinematicsSkew(z30) * (drc500dq.col(i) - dr300dq.col(i)),
                          dz30dq.col(i));
        setJacobianColumn(slice, 3,
                          kinematicsSkew(dz40dq.col(i)) * (rc500 - r400) + kinematicsSkew(z40) * (drc500dq.col(i) - dr400dq.col(i)),
                          dz40dq.col(i));
        setJacobianTensorSlice(dJc5dq, i, slice);

        // dJc6dq
        slice.setZero();
        setJacobianColumn(slice, 0,
                          kinematicsSkew(dz10dq.col(i)) * (rc600 - r100) + kinematicsSkew(z10) * (drc600dq.col(i) - dr100dq.col(i)),
                          dz10dq.col(i));
        setJacobianColumn(slice, 1,
                          kinematicsSkew(dz20dq.col(i)) * (rc600 - r200) + kinematicsSkew(z20) * (drc600dq.col(i) - dr200dq.col(i)),
                          dz20dq.col(i));
        setJacobianColumn(slice, 2,
                          kinematicsSkew(dz30dq.col(i)) * (rc600 - r300) + kinematicsSkew(z30) * (drc600dq.col(i) - dr300dq.col(i)),
                          dz30dq.col(i));
        setJacobianColumn(slice, 3,
                          kinematicsSkew(dz40dq.col(i)) * (rc600 - r400) + kinematicsSkew(z40) * (drc600dq.col(i) - dr400dq.col(i)),
                          dz40dq.col(i));
        setJacobianColumn(slice, 4,
                          kinematicsSkew(dz50dq.col(i)) * (rc600 - r500) + kinematicsSkew(z50) * (drc600dq.col(i) - dr500dq.col(i)),
                          dz50dq.col(i));
        setJacobianTensorSlice(dJc6dq, i, slice);

        // dJc7dq
        slice.setZero();
        setJacobianColumn(slice, 0,
                          kinematicsSkew(dz10dq.col(i)) * (rc700 - r100) + kinematicsSkew(z10) * (drc700dq.col(i) - dr100dq.col(i)),
                          dz10dq.col(i));
        setJacobianColumn(slice, 1,
                          kinematicsSkew(dz20dq.col(i)) * (rc700 - r200) + kinematicsSkew(z20) * (drc700dq.col(i) - dr200dq.col(i)),
                          dz20dq.col(i));
        setJacobianColumn(slice, 2,
                          kinematicsSkew(dz30dq.col(i)) * (rc700 - r300) + kinematicsSkew(z30) * (drc700dq.col(i) - dr300dq.col(i)),
                          dz30dq.col(i));
        setJacobianColumn(slice, 3,
                          kinematicsSkew(dz40dq.col(i)) * (rc700 - r400) + kinematicsSkew(z40) * (drc700dq.col(i) - dr400dq.col(i)),
                          dz40dq.col(i));
        setJacobianColumn(slice, 4,
                          kinematicsSkew(dz50dq.col(i)) * (rc700 - r500) + kinematicsSkew(z50) * (drc700dq.col(i) - dr500dq.col(i)),
                          dz50dq.col(i));
        setJacobianColumn(slice, 5,
                          kinematicsSkew(dz60dq.col(i)) * (rc700 - r600) + kinematicsSkew(z60) * (drc700dq.col(i) - dr600dq.col(i)),
                          dz60dq.col(i));
        setJacobianTensorSlice(dJc7dq, i, slice);

        // dJc8dq
        slice.setZero();
        setJacobianColumn(slice, 0,
                          kinematicsSkew(dz10dq.col(i)) * (rc800 - r100) + kinematicsSkew(z10) * (drc800dq.col(i) - dr100dq.col(i)),
                          dz10dq.col(i));
        setJacobianColumn(slice, 1,
                          kinematicsSkew(dz20dq.col(i)) * (rc800 - r200) + kinematicsSkew(z20) * (drc800dq.col(i) - dr200dq.col(i)),
                          dz20dq.col(i));
        setJacobianColumn(slice, 2,
                          kinematicsSkew(dz30dq.col(i)) * (rc800 - r300) + kinematicsSkew(z30) * (drc800dq.col(i) - dr300dq.col(i)),
                          dz30dq.col(i));
        setJacobianColumn(slice, 3,
                          kinematicsSkew(dz40dq.col(i)) * (rc800 - r400) + kinematicsSkew(z40) * (drc800dq.col(i) - dr400dq.col(i)),
                          dz40dq.col(i));
        setJacobianColumn(slice, 4,
                          kinematicsSkew(dz50dq.col(i)) * (rc800 - r500) + kinematicsSkew(z50) * (drc800dq.col(i) - dr500dq.col(i)),
                          dz50dq.col(i));
        setJacobianColumn(slice, 5,
                          kinematicsSkew(dz60dq.col(i)) * (rc800 - r600) + kinematicsSkew(z60) * (drc800dq.col(i) - dr600dq.col(i)),
                          dz60dq.col(i));
        setJacobianColumn(slice, 6,
                          kinematicsSkew(dz70dq.col(i)) * (rc800 - r700) + kinematicsSkew(z70) * (drc800dq.col(i) - dr700dq.col(i)),
                          dz70dq.col(i));
        setJacobianTensorSlice(dJc8dq, i, slice);
    }
}

void SystemKG3FullInput::calcMassMatrix(const Eigen::VectorXd &q)
{
    // Update M and dMdq based on current q.
    calcLinkGeometricJacobian(q);

    // Fill Mass Matrix
    Eigen::Matrix<double, 7, 7> M1, M2, M3, M4, M5, M6, M7, M8;
    Eigen::Matrix<double, 6, 6> b1, b2, b3, b4, b5, b6, b7, b8; // These are the 6x6 blocks of the mass matrix, M = Jc1.'*b*Jc1

    b1.block<3, 3>(0, 0) = m1 * Eigen::Matrix3d::Identity(); // Mass of link 1
    b1.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    b1.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    b1.block<3, 3>(3, 3) = R01 * Ic11 * R01.transpose(); // Inertia of link 1

    b2.block<3, 3>(0, 0) = m2 * Eigen::Matrix3d::Identity(); // Mass of link 2
    b2.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    b2.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    b2.block<3, 3>(3, 3) = R02 * Ic22 * R02.transpose(); // Inertia of link 2

    b3.block<3, 3>(0, 0) = m3 * Eigen::Matrix3d::Identity(); // Mass of link 3
    b3.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    b3.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    b3.block<3, 3>(3, 3) = R03 * Ic33 * R03.transpose(); // Inertia of link 3

    b4.block<3, 3>(0, 0) = m4 * Eigen::Matrix3d::Identity(); // Mass of link 4
    b4.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    b4.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    b4.block<3, 3>(3, 3) = R04 * Ic44 * R04.transpose(); // Inertia of link 4

    b5.block<3, 3>(0, 0) = m5 * Eigen::Matrix3d::Identity(); // Mass of link 5
    b5.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    b5.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    b5.block<3, 3>(3, 3) = R05 * Ic55 * R05.transpose(); // Inertia of link 5

    b6.block<3, 3>(0, 0) = m6 * Eigen::Matrix3d::Identity(); // Mass of link 6
    b6.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    b6.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    b6.block<3, 3>(3, 3) = R06 * Ic66 * R06.transpose(); // Inertia of link 6

    b7.block<3, 3>(0, 0) = m7 * Eigen::Matrix3d::Identity(); // Mass of link 7
    b7.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    b7.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    b7.block<3, 3>(3, 3) = R07 * Ic77 * R07.transpose(); // Inertia of link 7

    b8.block<3, 3>(0, 0) = m8 * Eigen::Matrix3d::Identity(); // Mass of link 8
    b8.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    b8.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    b8.block<3, 3>(3, 3) = R08 * Ic88 * R08.transpose(); // Inertia of link 8

    // Fill the mass matrix M
    M1 = Jc1.transpose() * b1 * Jc1;
    M2 = Jc2.transpose() * b2 * Jc2;
    M3 = Jc3.transpose() * b3 * Jc3;
    M4 = Jc4.transpose() * b4 * Jc4;
    M5 = Jc5.transpose() * b5 * Jc5;
    M6 = Jc6.transpose() * b6 * Jc6;
    M7 = Jc7.transpose() * b7 * Jc7;
    M8 = Jc8.transpose() * b8 * Jc8;

    M = M1 + M2 + M3 + M4 + M5 + M6 + M7 + M8; // Total mass matrix
}

void SystemKG3FullInput::calcCoriolisMatrix(const Eigen::VectorXd &q, const Eigen::VectorXd &dqdt)
{
    // Update C based on current q and dqdt.
    // Calculate Geometric Hessian
    calcLinkGeometricHessian(q);

    // For each of the 7 slices, copy the 3x3 block (rows 0-2, cols 0-2) into the new tensor.
    for (int k = 0; k < 7; ++k)
    {
        Eigen::array<Eigen::Index, 3> offset = {0, 0, k};
        Eigen::array<Eigen::Index, 3> extent = {3, 3, 1};
        dR01dq.slice(offset, extent) = dA01dq.slice(offset, extent);
        dR02dq.slice(offset, extent) = dA02dq.slice(offset, extent);
        dR03dq.slice(offset, extent) = dA03dq.slice(offset, extent);
        dR04dq.slice(offset, extent) = dA04dq.slice(offset, extent);
        dR05dq.slice(offset, extent) = dA05dq.slice(offset, extent);
        dR06dq.slice(offset, extent) = dA06dq.slice(offset, extent);
        dR07dq.slice(offset, extent) = dA07dq.slice(offset, extent);
        dR08dq.slice(offset, extent) = dA08dq.slice(offset, extent);
    }

    // Resize dMdq vector to hold 7 matrices
    dMdq.resize(7);

// For each joint
#pragma omp parallel for
    for (int k = 0; k < 7; ++k)
    {
        // Initialize the k-th slice
        dMdq[k] = Eigen::Matrix<double, 7, 7>::Zero();

        // Get Jacobian derivatives for this slice
        Eigen::Matrix<double, 6, 7> dJc1_k, dJc2_k, dJc3_k, dJc4_k, dJc5_k, dJc6_k, dJc7_k, dJc8_k;
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < 7; ++j)
            {
                dJc1_k(i, j) = dJc1dq(i, j, k);
                dJc2_k(i, j) = dJc2dq(i, j, k);
                dJc3_k(i, j) = dJc3dq(i, j, k);
                dJc4_k(i, j) = dJc4dq(i, j, k);
                dJc5_k(i, j) = dJc5dq(i, j, k);
                dJc6_k(i, j) = dJc6dq(i, j, k);
                dJc7_k(i, j) = dJc7dq(i, j, k);
                dJc8_k(i, j) = dJc8dq(i, j, k);
            }
        }

        // Get rotation matrix derivatives for this slice
        Eigen::Matrix3d dR01_k, dR02_k, dR03_k, dR04_k, dR05_k, dR06_k, dR07_k, dR08_k;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                dR01_k(i, j) = dR01dq(i, j, k);
                dR02_k(i, j) = dR02dq(i, j, k);
                dR03_k(i, j) = dR03dq(i, j, k);
                dR04_k(i, j) = dR04dq(i, j, k);
                dR05_k(i, j) = dR05dq(i, j, k);
                dR06_k(i, j) = dR06dq(i, j, k);
                dR07_k(i, j) = dR07dq(i, j, k);
                dR08_k(i, j) = dR08dq(i, j, k);
            }
        }

        // Create mass and inertia blocks
        Eigen::Matrix<double, 6, 6> b1, b2, b3, b4, b5, b6, b7, b8;
        Eigen::Matrix<double, 6, 6> db1, db2, db3, db4, db5, db6, db7, db8;

        // Mass blocks
        b1.setZero();
        b1.block<3, 3>(0, 0) = m1 * Eigen::Matrix3d::Identity();
        b1.block<3, 3>(3, 3) = R01 * Ic11 * R01.transpose();
        b2.setZero();
        b2.block<3, 3>(0, 0) = m2 * Eigen::Matrix3d::Identity();
        b2.block<3, 3>(3, 3) = R02 * Ic22 * R02.transpose();
        b3.setZero();
        b3.block<3, 3>(0, 0) = m3 * Eigen::Matrix3d::Identity();
        b3.block<3, 3>(3, 3) = R03 * Ic33 * R03.transpose();
        b4.setZero();
        b4.block<3, 3>(0, 0) = m4 * Eigen::Matrix3d::Identity();
        b4.block<3, 3>(3, 3) = R04 * Ic44 * R04.transpose();
        b5.setZero();
        b5.block<3, 3>(0, 0) = m5 * Eigen::Matrix3d::Identity();
        b5.block<3, 3>(3, 3) = R05 * Ic55 * R05.transpose();
        b6.setZero();
        b6.block<3, 3>(0, 0) = m6 * Eigen::Matrix3d::Identity();
        b6.block<3, 3>(3, 3) = R06 * Ic66 * R06.transpose();
        b7.setZero();
        b7.block<3, 3>(0, 0) = m7 * Eigen::Matrix3d::Identity();
        b7.block<3, 3>(3, 3) = R07 * Ic77 * R07.transpose();
        b8.setZero();
        b8.block<3, 3>(0, 0) = m8 * Eigen::Matrix3d::Identity();
        b8.block<3, 3>(3, 3) = R08 * Ic88 * R08.transpose();

        // Derivative blocks
        db1.setZero();
        db1.block<3, 3>(3, 3) = dR01_k * Ic11 * R01.transpose() + R01 * Ic11 * dR01_k.transpose();
        db2.setZero();
        db2.block<3, 3>(3, 3) = dR02_k * Ic22 * R02.transpose() + R02 * Ic22 * dR02_k.transpose();
        db3.setZero();
        db3.block<3, 3>(3, 3) = dR03_k * Ic33 * R03.transpose() + R03 * Ic33 * dR03_k.transpose();
        db4.setZero();
        db4.block<3, 3>(3, 3) = dR04_k * Ic44 * R04.transpose() + R04 * Ic44 * dR04_k.transpose();
        db5.setZero();
        db5.block<3, 3>(3, 3) = dR05_k * Ic55 * R05.transpose() + R05 * Ic55 * dR05_k.transpose();
        db6.setZero();
        db6.block<3, 3>(3, 3) = dR06_k * Ic66 * R06.transpose() + R06 * Ic66 * dR06_k.transpose();
        db7.setZero();
        db7.block<3, 3>(3, 3) = dR07_k * Ic77 * R07.transpose() + R07 * Ic77 * dR07_k.transpose();
        db8.setZero();
        db8.block<3, 3>(3, 3) = dR08_k * Ic88 * R08.transpose() + R08 * Ic88 * dR08_k.transpose();

        // Accumulate contributions from each link
        dMdq[k] = dJc1_k.transpose() * b1 * Jc1 + Jc1.transpose() * db1 * Jc1 + Jc1.transpose() * b1 * dJc1_k + dJc2_k.transpose() * b2 * Jc2 + Jc2.transpose() * db2 * Jc2 + Jc2.transpose() * b2 * dJc2_k + dJc3_k.transpose() * b3 * Jc3 + Jc3.transpose() * db3 * Jc3 + Jc3.transpose() * b3 * dJc3_k + dJc4_k.transpose() * b4 * Jc4 + Jc4.transpose() * db4 * Jc4 + Jc4.transpose() * b4 * dJc4_k + dJc5_k.transpose() * b5 * Jc5 + Jc5.transpose() * db5 * Jc5 + Jc5.transpose() * b5 * dJc5_k + dJc6_k.transpose() * b6 * Jc6 + Jc6.transpose() * db6 * Jc6 + Jc6.transpose() * b6 * dJc6_k + dJc7_k.transpose() * b7 * Jc7 + Jc7.transpose() * db7 * Jc7 + Jc7.transpose() * b7 * dJc7_k + dJc8_k.transpose() * b8 * Jc8 + Jc8.transpose() * db8 * Jc8 + Jc8.transpose() * b8 * dJc8_k;
    }

    Gamma.setZero(); // Initialize Gamma to zero
    // Compute Christoffel symbols
    for (int i = 0; i < 7; ++i)
    {
        for (int j = 0; j < 7; ++j)
        {
            for (int k = 0; k < 7; ++k)
            {
                // Note: dMdq[k](j,i) corresponds to MATLAB's dMdq(k,j,i)
                Gamma(i, j, k) = (dMdq[k](j, i) + dMdq[k](i, j) - dMdq[i](j, k)) / 2.0;
            }
        }
    }

    // Initialize Coriolis matrix
    C.setZero();

    // Compute Coriolis matrix
    for (int k = 0; k < 7; ++k)
    {
        for (int j = 0; j < 7; ++j)
        {
            for (int i = 0; i < 7; ++i)
            {
                C(k, j) += Gamma(i, j, k) * dqdt(i);
            }
        }
    }
}

void SystemKG3FullInput::calcDampingMatrix(const Eigen::VectorXd &dqdt)
{
    // Update D based on current dqdt.
    Eigen::Matrix<double, 7, 1> diag;
    diag << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1; // Example diagonal values for damping matrix
    D.diagonal() = diag;
}

void SystemKG3FullInput::calcGravityTorque(const Eigen::VectorXd &q)
{
    // Update gq based on current q.

    Eigen::Vector3d g0;
    g0 << 0, 0, g; // Gravity vector in base frame

    Vec6d tauc1, tauc2, tauc3, tauc4, tauc5, tauc6, tauc7, tauc8;
    tauc1.setZero();         // Initialize gravity torque vector for link 1
    tauc1.head(3) = m1 * g0; // Gravity torque for link 1
    tauc2.setZero();         // Initialize gravity torque vector for link 2
    tauc2.head(3) = m2 * g0; // Gravity torque for link 2
    tauc3.setZero();         // Initialize gravity torque vector for link 3
    tauc3.head(3) = m3 * g0; // Gravity torque for link 3
    tauc4.setZero();         // Initialize gravity torque vector for link 4
    tauc4.head(3) = m4 * g0; // Gravity torque for link 4
    tauc5.setZero();         // Initialize gravity torque vector for link 5
    tauc5.head(3) = m5 * g0; // Gravity torque for link 5
    tauc6.setZero();         // Initialize gravity torque vector for link 6
    tauc6.head(3) = m6 * g0; // Gravity torque for link 6
    tauc7.setZero();         // Initialize gravity torque vector for link 7
    tauc7.head(3) = m7 * g0; // Gravity torque for link 7
    tauc8.setZero();         // Initialize gravity torque vector for link 8
    tauc8.head(3) = m8 * g0; // Gravity torque for link 8

    // Compute gravity torque vector
    gq = Jc1.transpose() * tauc1 +
         Jc2.transpose() * tauc2 +
         Jc3.transpose() * tauc3 +
         Jc4.transpose() * tauc4 +
         Jc5.transpose() * tauc5 +
         Jc6.transpose() * tauc6 +
         Jc7.transpose() * tauc7 +
         Jc8.transpose() * tauc8;
}

Eigen::Matrix4d SystemKG3FullInput::kinematicsTranx(const double &q)
{
    Eigen::Matrix4d A;
    A << 1, 0, 0, q,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d SystemKG3FullInput::kinematicsTrany(const double &q)
{
    Eigen::Matrix4d A;
    A << 1, 0, 0, 0,
        0, 1, 0, q,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d SystemKG3FullInput::kinematicsTranz(const double &q)
{
    Eigen::Matrix4d A;
    A << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, q,
        0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d SystemKG3FullInput::kinematicsRotx(const double &q)
{
    Eigen::Matrix4d A;
    A << 1, 0, 0, 0,
        0, std::cos(q), -std::sin(q), 0,
        0, std::sin(q), std::cos(q), 0,
        0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d SystemKG3FullInput::kinematicsRotz(const double &q)
{
    Eigen::Matrix4d A;
    A << std::cos(q), -std::sin(q), 0, 0,
        std::sin(q), std::cos(q), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d SystemKG3FullInput::kinematicsRoty(const double &q)
{
    Eigen::Matrix4d A;
    A << std::cos(q), 0, std::sin(q), 0,
        0, 1, 0, 0,
        -std::sin(q), 0, std::cos(q), 0,
        0, 0, 0, 1;
    return A;
}

Eigen::Matrix3d SystemKG3FullInput::kinematicsSkew(const Eigen::Vector3d &u)
{
    Eigen::Matrix3d S;
    S << 0, -u(2), u(1),
        u(2), 0, -u(0),
        -u(1), u(0), 0;
    return S;
}

Eigen::Matrix4d SystemKG3FullInput::kinematicsHatSE3(const Eigen::VectorXd &x)
{
    Eigen::Matrix4d G = Eigen::Matrix4d::Zero();
    G.block<3, 3>(0, 0) = kinematicsSkew(x.tail(3));
    G.block<3, 1>(0, 3) = x.head(3);
    G.row(3) = Eigen::Vector4d::Zero();
    return G;
}

Eigen::MatrixXd SystemKG3FullInput::compute_dMdt(const std::vector<Eigen::MatrixXd> &dMdq, const Eigen::VectorXd &dqdt)
{
    if (dMdq.empty() || dMdq.size() != 7)
    {
        return Eigen::MatrixXd::Zero(7, 7);
    }

    // Initialize result matrix
    Eigen::MatrixXd dMdt = Eigen::MatrixXd::Zero(7, 7);

    // Sum over all slices
    for (int k = 0; k < 7; ++k)
    {
        dMdt += dMdq[k] * dqdt(k); // Multiply each slice by corresponding dqdt element
    }

    return dMdt;
}

Eigen::MatrixXd SystemKG3FullInput::compute_dJAdt(const std::vector<Eigen::Matrix<double, 6, 7>> &HA, const Eigen::VectorXd &dqdt)
{
    // Initialize result matrix with zeros
    Eigen::MatrixXd dJAdt = Eigen::MatrixXd::Zero(6, 7);

    if (HA.empty() || HA.size() != 7)
    {
        return dJAdt; // Return zero matrix if input is invalid
    }

    for (int k = 0; k < 7; ++k)
    {
        dJAdt += HA[k] * dqdt(k); // Multiply each slice by corresponding dqdt element
    }

    return dJAdt;
}

void SystemKG3FullInput::setTensorSlice(Eigen::Tensor<double, 3> &tensor, int slice, const Eigen::Matrix4d &matrix)
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            tensor(i, j, slice) = matrix(i, j);
        }
    }
}

// Helper function to create a 6x7 zero matrix with a 3x1 column at specified position
void SystemKG3FullInput::setJacobianColumn(Eigen::Matrix<double, 6, 7> &J, int col,
                                           const Eigen::Vector3d &top, const Eigen::Vector3d &bottom)
{
    J.block<3, 1>(0, col) = top;
    J.block<3, 1>(3, col) = bottom;
}

// Helper function to set a 6x7 matrix into a tensor slice
void SystemKG3FullInput::setJacobianTensorSlice(Eigen::Tensor<double, 3> &tensor, int slice, const Eigen::Matrix<double, 6, 7> &matrix)
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 7; ++j)
        {
            tensor(i, j, slice) = matrix(i, j);
        }
    }
}

Eigen::VectorXd SystemKG3FullInput::computeInput(const Eigen::VectorXd &x, const Eigen::VectorXd &Fext, const Vec6d &xestar_in)
{
    // Extract joint velocities (dqdt) and positions (q) from the state vector.
    Eigen::VectorXd dqdt = x.segment(0, 7);
    Eigen::VectorXd q = x.segment(7, 7);

    // Update system state and compute necessary matrices
    calcEffectorAnalyticalHessian(q);
    Eigen::VectorXd xe = this->xe;
    Eigen::MatrixXd JA = this->JA;

    // Helper function to compute dJAdt can be a private member function
    Eigen::MatrixXd dJAdt = compute_dJAdt(this->HA, dqdt);

    calcMassMatrix(q);
    Eigen::MatrixXd M = this->M;

    calcCoriolisMatrix(q, dqdt);
    Eigen::MatrixXd C = this->C;

    calcDampingMatrix(dqdt);
    Eigen::MatrixXd D = this->D;

    calcGravityTorque(q);
    Eigen::VectorXd g = this->gq;

    // Get the closed-loop effector matrices
    Eigen::MatrixXd dMdt = compute_dMdt(this->dMdq, dqdt);

    // Check for singularities
    if (M.fullPivLu().rank() < 5)
    {
        std::cerr << "Mass matrix M rank deficiency detected." << std::endl;
    }
    if (JA.fullPivLu().rank() < 5)
    {
        std::cerr << "Warning: Robot is in singular configuration." << std::endl;
        return Eigen::VectorXd::Zero(7);
    }

    // Control law
    Eigen::MatrixXd A = Me * JA;
    Eigen::VectorXd RHS = Fext - Ke * (xe - xestar_in) - (De * JA + Me * dJAdt) * dqdt + Me * xestardt + De * xestarddt;

    Eigen::VectorXd qdd = A.colPivHouseholderQr().solve(RHS);
    Eigen::VectorXd u = M * qdd + C * dqdt + D * dqdt + g;

    return u;
}

// Operational space inverse dynamics controller
// Assumes dxestardt and ddxestarddt are zero for now
Eigen::VectorXd SystemKG3FullInput::opSpaceControl(const Eigen::VectorXd &x, const Vec6d &xestar_in, const double KP_aggro, const double KD_aggro)
{
    // Extract joint velocities (dqdt) and positions (q) from the state vector.
    Eigen::VectorXd dqdt = x.segment(0, 7);
    Eigen::VectorXd q = x.segment(7, 7);

    // Update system state and compute necessary matrices
    calcEffectorAnalyticalHessian(q);
    Eigen::VectorXd xe = this->xe;
    Eigen::MatrixXd JA = this->JA;

    // Helper function to compute dJAdt can be a private member function
    Eigen::MatrixXd dJAdt = compute_dJAdt(this->HA, dqdt);

    calcMassMatrix(q);
    Eigen::MatrixXd M = this->M;

    calcCoriolisMatrix(q, dqdt);
    Eigen::MatrixXd C = this->C;

    calcDampingMatrix(dqdt);
    Eigen::MatrixXd D = this->D;

    calcGravityTorque(q);
    Eigen::VectorXd g = this->gq;

    // Get the closed-loop effector matrices
    Eigen::MatrixXd dMdt = compute_dMdt(this->dMdq, dqdt);

    // Check for singularities
    if (M.fullPivLu().rank() < 5)
    {
        std::cerr << "Mass matrix M rank deficiency detected." << std::endl;
    }
    if (JA.fullPivLu().rank() < 5)
    {
        std::cerr << "Warning: Robot is in singular configuration." << std::endl;
        return Eigen::VectorXd::Zero(7);
    }


    
    Eigen::VectorXd RHS = xestarddt + KD_aggro * KD * (xestarddt - JA*dqdt) +  KP_aggro * KP * (xestar_in - xe) - dJAdt*dqdt;
    // Control law
    Eigen::VectorXd qdd = JA.colPivHouseholderQr().solve(RHS);
    Eigen::VectorXd u = M * qdd + C * dqdt + D * dqdt + g;
    return u;
}

// Integral Action operational space controller
// Assumes dxestardt and ddxestarddt are zero for now
Eigen::VectorXd SystemKG3FullInput::opSpaceControl_IA(const Eigen::VectorXd &x, const Vec6d  const double KP_aggro, const double KD_aggro, const double KI_aggro)
{
    // Extract joint velocities (dqdt) and positions (q) from the state vector.
    Eigen::VectorXd dqdt = x.segment(0, 7);
    Eigen::VectorXd q = x.segment(7, 7);

    // Update system state and compute necessary matrices
    calcEffectorAnalyticalHessian(q);
    Eigen::VectorXd xe = this->xe;
    Eigen::MatrixXd JA = this->JA;

    // Helper function to compute dJAdt can be a private member function
    Eigen::MatrixXd dJAdt = compute_dJAdt(this->HA, dqdt);

    calcMassMatrix(q);
    Eigen::MatrixXd M = this->M;

    calcCoriolisMatrix(q, dqdt);
    Eigen::MatrixXd C = this->C;

    calcDampingMatrix(dqdt);
    Eigen::MatrixXd D = this->D;

    calcGravityTorque(q);
    Eigen::VectorXd g = this->gq;

    // Get the closed-loop effector matrices
    Eigen::MatrixXd dMdt = compute_dMdt(this->dMdq, dqdt);

    // Check for singularities
    if (M.fullPivLu().rank() < 5)
    {
        std::cerr << "Mass matrix M rank deficiency detected." << std::endl;
    }
    if (JA.fullPivLu().rank() < 5)
    {
        std::cerr << "Warning: Robot is in singular configuration." << std::endl;
        return Eigen::VectorXd::Zero(7);
    }

    double dt = 0.001;
    integralError = integralError + (xestar_in - xe)*dt;

    // Anit-windup for the integral error
    for (int i=0; i<6; i++) {
        if (integralError(i) > integral_max(i)) {
            integralError(i) = integral_max(i);
        } else if (integralError(i) < -integral_max(i)) {
            integralError(i) = -integral_max(i);
        }
    }


    
    Eigen::VectorXd RHS = xestarddt + KD_aggro * KD * (xestarddt - JA*dqdt) +  KP_aggro * KP * (xestar_in - xe) + KI_aggro * KI * integralError - dJAdt*dqdt;
    // Control law
    Eigen::VectorXd qdd = JA.colPivHouseholderQr().solve(RHS);
    Eigen::VectorXd u = M * qdd + C * dqdt + D * dqdt + g;
    return u;
}

// // Integral Action operational space controller INCLUDING VELOCITY AND ACCLERATION TRACKING
// HAS NOT BEEN VERIFIED AS OF 12/11/2025
// Eigen::VectorXd SystemKG3FullInput::opSpaceControl_IA(const Eigen::VectorXd &x, const Vec6d &xestar_in, const Vec6d &xestar_in, const Vec6d &xestardt, const double KP_aggro, const double KD_aggro, const double KI_aggro)
// {
//     // Extract joint velocities (dqdt) and positions (q) from the state vector.
//     Eigen::VectorXd dqdt = x.segment(0, 7);
//     Eigen::VectorXd q = x.segment(7, 7);

//     // Update system state and compute necessary matrices
//     calcEffectorAnalyticalHessian(q);
//     Eigen::VectorXd xe = this->xe;
//     Eigen::MatrixXd JA = this->JA;

//     // Helper function to compute dJAdt can be a private member function
//     Eigen::MatrixXd dJAdt = compute_dJAdt(this->HA, dqdt);

//     calcMassMatrix(q);
//     Eigen::MatrixXd M = this->M;

//     calcCoriolisMatrix(q, dqdt);
//     Eigen::MatrixXd C = this->C;

//     calcDampingMatrix(dqdt);
//     Eigen::MatrixXd D = this->D;

//     calcGravityTorque(q);
//     Eigen::VectorXd g = this->gq;

//     // Get the closed-loop effector matrices
//     Eigen::MatrixXd dMdt = compute_dMdt(this->dMdq, dqdt);

//     // Check for singularities
//     if (M.fullPivLu().rank() < 5)
//     {
//         std::cerr << "Mass matrix M rank deficiency detected." << std::endl;
//     }
//     if (JA.fullPivLu().rank() < 5)
//     {
//         std::cerr << "Warning: Robot is in singular configuration." << std::endl;
//         return Eigen::VectorXd::Zero(7);
//     }

//     double dt = 0.001; // Sampling rate of main loop. Would be better if this was extracted directly from the main loop
//     integralError = integralError + (xestar_in - xe)*dt;

//     // Anit-windup for the integral error
//     for (int i=0; i<6; i++) {
//         if (integralError(i) > integral_max(i)) {
//             integralError(i) = integral_max(i);
//         } else if (integralError(i) < -integral_max(i)) {
//             integralError(i) = -integral_max(i);
//         }
//     }


    
//     Eigen::VectorXd RHS = xestarddt + KD_aggro * KD * (xestardt - JA*dqdt) +  KP_aggro * KP * (xestar_in - xe) + KI_aggro * KI * integralError - dJAdt*dqdt;
//     // Control law
//     Eigen::VectorXd qdd = JA.colPivHouseholderQr().solve(RHS);
//     Eigen::VectorXd u = M * qdd + C * dqdt + D * dqdt + g;
//     return u;
// }


// Joint space inverse dynamics controller
Eigen::VectorXd SystemKG3FullInput::jointSpaceControl(const Eigen::VectorXd &x, const Vec6d &qstar_in, const double KP_aggro, const double KD_aggro)
{
    // Extract joint velocities (dqdt) and positions (q) from the state vector.
    Eigen::VectorXd dqdt = x.segment(0, 7);
    Eigen::VectorXd q = x.segment(7, 7);

    // Update system state and compute necessary matrices
    calcEffectorAnalyticalHessian(q);
    Eigen::VectorXd xe = this->xe;
    Eigen::MatrixXd JA = this->JA;

    // Helper function to compute dJAdt can be a private member function
    Eigen::MatrixXd dJAdt = compute_dJAdt(this->HA, dqdt);

    calcMassMatrix(q);
    Eigen::MatrixXd M = this->M;

    calcCoriolisMatrix(q, dqdt);
    Eigen::MatrixXd C = this->C;

    calcDampingMatrix(dqdt);
    Eigen::MatrixXd D = this->D;

    calcGravityTorque(q);
    Eigen::VectorXd g = this->gq;

    // Get the closed-loop effector matrices
    Eigen::MatrixXd dMdt = compute_dMdt(this->dMdq, dqdt);

    // Check for singularities
    if (M.fullPivLu().rank() < 5)
    {
        std::cerr << "Mass matrix M rank deficiency detected." << std::endl;
    }
    if (JA.fullPivLu().rank() < 5)
    {
        std::cerr << "Warning: Robot is in singular configuration." << std::endl;
        return Eigen::VectorXd::Zero(7);
    }

    // Compute the joint position error
    Eigen::VectorXd qe = computeAngularError(qstar_in, q);

    // Control law
    Eigen::VectorXd qdd = KD_aggro * KD_grav * (-dqdt) + KP_aggro * KP_grav * qe;
    Eigen::VectorXd u = M * qdd + C * dqdt + D * dqdt + g;
    return u;
}


// Gravity compensation PD controller 
Eigen::VectorXd SystemKG3FullInput::gravityCompensation(const Eigen::VectorXd &x, const Eigen::VectorXd &qstar_in, const double aggro)
{
    // Extract joint velocities (dqdt) and positions (q) from the state vector.
    Eigen::VectorXd dqdt = x.segment(0, 7);
    Eigen::VectorXd q = x.segment(7, 7);

    // Update system state and compute necessary matrices
    calcEffectorAnalyticalHessian(q);
    Eigen::VectorXd xe = this->xe;
    Eigen::MatrixXd JA = this->JA;

    // Helper function to compute dJAdt can be a private member function
    Eigen::MatrixXd dJAdt = compute_dJAdt(this->HA, dqdt);

    calcMassMatrix(q);
    Eigen::MatrixXd M = this->M;

    calcCoriolisMatrix(q, dqdt);
    Eigen::MatrixXd C = this->C;

    calcDampingMatrix(dqdt);
    Eigen::MatrixXd D = this->D;

    calcGravityTorque(q);
    Eigen::VectorXd g = this->gq;

    // Get the closed-loop effector matrices
    Eigen::MatrixXd dMdt = compute_dMdt(this->dMdq, dqdt);

    // Check for singularities
    if (M.fullPivLu().rank() < 5)
    {
        std::cerr << "Mass matrix M rank deficiency detected." << std::endl;
    }
    if (JA.fullPivLu().rank() < 5)
    {
        std::cerr << "Warning: Robot is in singular configuration." << std::endl;
        return Eigen::VectorXd::Zero(7);
    }

    // Compute the joint position error
    Eigen::VectorXd qe = computeAngularError(qstar_in, q);

    // Gravity compensation controller
    Eigen::VectorXd u = g + aggro*KP_grav * qe - KD_grav * dqdt;

    return u;
}




/////  Helper Functions for angle wrapping /////
double SystemKG3FullInput::wrapTo2Pi(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

// Wraps an Eigen vector of angles to [0,2pi]
Eigen::VectorXd SystemKG3FullInput::wrapTo2Pi(const Eigen::VectorXd &angles)
{
    Eigen::VectorXd wrapped(angles.size());
    for (int i = 0; i < angles.size(); i++)
    {
        wrapped(i) = wrapTo2Pi(angles(i));
    }
    return wrapped;
}

// Computes wrapped angular error for a single angle
double SystemKG3FullInput::computeAngularError(double target, double current)
{
    double error = target - current;

    // Wrap to [0,2pi]
    error = fmod(error, 2 * M_PI);
    if (error < 0)
    {
        error += 2 * M_PI;
    }

    if (error > M_PI)
    {
        error -= 2 * M_PI;
    }

    return error;
}

// Computes wrapped angular error for Eigen vectors (multi-joint)
Eigen::VectorXd SystemKG3FullInput::computeAngularError(const Eigen::VectorXd &target, const Eigen::VectorXd &current)
{
    Eigen::VectorXd error(target.size());
    for (int i = 0; i < target.size(); i++)
    {
        error(i) = computeAngularError(target(i), current(i));
    }
    return error;
}

