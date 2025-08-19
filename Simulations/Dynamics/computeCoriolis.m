function C = computeCoriolis(myRobot, q, dq)
% Numerically compute the Coriolis matrix C(q,dq) for an n-DOF manipulator
%
% Inputs:
%   myRobot - object/struct with mass matrix function: myRobot.M(q)
%   q       - 1xn vector of joint positions
%   dq      - 1xn vector of joint velocities
%
% Output:
%   C       - nxn Coriolis/centrifugal matrix

    n = length(q);
    delta = 1e-8;           % finite difference step
    M0 = myRobot.M;          % original mass matrix at q
    
    % If M is a function handle
    if isa(M0,'function_handle')
        M0 = M0(q);
    end

    % Preallocate 3D array for derivatives
    dM = zeros(n,n,n);  % dM(:,:,k) = dM/dq_k

    % Compute numerical derivatives of M w.r.t each joint
    for k = 1:n
        dqvec = zeros(1,n);
        dqvec(k) = delta;

        % central difference
        if isa(M0,'function_handle')
            M_plus  = myRobot.M(q + dqvec);
            M_minus = myRobot.M(q - dqvec);
        else
            % assume M0 is already the numeric matrix at q
            myRobot.q = q + dqvec;
            myRobot = myRobot.MassMatrix(q); % implement if needed
            M_plus = myRobot.M;
            myRobot.q = q - dqvec;
            myRobot = myRobot.MassMatrix(q);
            M_minus = myRobot.M;
        end

        dM(:,:,k) = (M_plus - M_minus)/(2*delta);
    end

    % Compute Coriolis matrix
    C = zeros(n,n);
    for i = 1:n
        for j = 1:n
            c_ij = 0;
            for k = 1:n
                c_ij = c_ij + 0.5 * ( dM(i,j,k) + dM(i,k,j) - dM(k,j,i) ) * dq(k);
            end
            C(i,j) = c_ij;
        end
    end
end
