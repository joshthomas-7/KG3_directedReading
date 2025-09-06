%%% Joshua Thomas
%%% C3376353

classdef KG3 < handle
    % This class stores the kinematic and dynamic properties of the KG3.
    % It has methods to compute the:
    % - Forward kinematics
    % - Inverse kinematics
    % - Dynamic model parameters (mass matrix, coriolis matrix, gravity
    % torque)
    % - Implmenets motion planning
    % - Implements motion control using inverse dynamics 

    properties
        %% FKM properties
        Ts
        T_all % all transformation matricies for the KG3 T0i (referred to base frame).
        Aij_all % local transformations between matricies

        pose  % pose of the KG3.

        r_COM

        %% IKM Properties
        W
        V
        rhoVec


        %% Differential Kinematics Properties
        J % geometric jacobian.
        JA  % Analytical Jacobian
        H   % Analytical Hessian

        qdot
        dt
        
        %% Dynamics Properties
        mArr %kg, array each stores the mass of each link in the KG3.
        I    % cells which store the inertia tensors of each link referred to the COM.
        q     % current joint configuration
        M     % mass matrix 
        C     % Coriolis matrix
        G     % Gravity torque
        gravity = [0; 0; -9.81];    %m/s^2 default gravity vector 

        %% Control Properties
        KD
        KP

    end

    methods
        function obj = KG3()
            %KG3 Construct an instance of this class
            % Precomputes the homogenous transformations between each frame
            % of the KG3 
            [obj.Ts, obj.mArr, obj.I, obj.r_COM, obj.KD, obj.KP] = KG3.preCompute();
        end


        %% Kinematics
        function obj = FKM(obj, q)
            % Computes the forward kinematics of the KG3 for a given joint configuration (q)
            obj.q = q;
            N = length(q);  % 7 joints
            L = length(obj.Ts); % 8 links including end-effector
            xi = [0;0;0;0;0;1]; % constant local twist for each joint (all joints revolute)

            % Build rotation matrices for each joint (for clarity, optional)
            Rs = cell(1,N);
            for i = 1:N
                Rs{i} = [cos(q(i)) -sin(q(i)) 0 0;
                         sin(q(i))  cos(q(i)) 0 0;
                         0           0        1 0;
                         0           0        0 1];
            end

            % For joint links 1 to N
            T_base = eye(4);
            obj.T_all = cell(1,N+1); % 8 links
            for j = 1:N
                A = obj.Ts{j} * Rs{j};
                T_base = T_base * A;
                obj.T_all{j} = T_base;
                obj.Aij_all{j} = A;

                % Computing the derivative of each Ai-1_i transformation
                % obj.dAdqi_all{j} = KG3.hatSE3(xi)*A;   
            end
            % End-effector
            T_base = T_base * obj.Ts{N+1};
            obj.T_all{N+1} = T_base;
            obj.Aij_all{N+1} = obj.Ts{N+1};

        end

        function obj = computeJacobians(obj)
            % Computes the geometric Jacobians for the KG3 manipulator
            % Format: [linear; angular] - first 3 rows are linear, last 3 are angular
            
            N = length(obj.q);  % 7 joints
            L = length(obj.Ts); % 8 links including end-effector
            
            % Initialize Jacobians
            obj.J = zeros(6, N);           % End-effector Jacobian
            
            % End-effector position
            T_ee = obj.T_all{N+1};
            p_ee = T_ee(1:3, 4);
            
            %% Compute End-effector Jacobian
            % For each joint, compute its contribution to the end-effector motion
            % Joint i rotates about its local z-axis in the frame where it's mounted
            
            for i = 1:N
                if i == 1
                    % Joint 1: The axis should be extracted from the T01 transformation
                    % Joint 1 rotates about the z-axis defined by T01
                    T_frame = obj.Ts{1};  % T01
                    z_i = T_frame(1:3, 3);  % z-axis from T01 = [0, 0, -1]
                    o_i = T_frame(1:3, 4);  % origin from T01
                else
                    % For joint i > 1, we need to find the frame where joint i is located
                    % This is the frame after all previous joints have moved, plus Ts{i}
                    T_accum = eye(4);
                    for j = 1:i-1
                        % Apply all previous joint transformations
                        R_j = [cos(obj.q(j)) -sin(obj.q(j)) 0 0;
                               sin(obj.q(j))  cos(obj.q(j)) 0 0;
                               0               0            1 0;
                               0               0            0 1];
                        A_j = obj.Ts{j} * R_j;
                        T_accum = T_accum * A_j;
                    end
                    % Add the constant part of transformation i
                    T_frame = T_accum * obj.Ts{i};
                    z_i = T_frame(1:3, 3);  % z-axis where joint i rotates
                    o_i = T_frame(1:3, 4);  % origin where joint i rotates
                end
                
                % Compute Jacobian column for joint i
                % MATLAB convention: [angular; linear]
                obj.J(1:3, i) = z_i;  % Angular velocity part
                obj.J(4:6, i) = cross(z_i, p_ee - o_i);  % Linear velocity part
            end
        end
 
        function obj = computeAnalyticalJacobian(obj)
            % Computes the analytical jacobian, in the current
            % configuration

            R = obj.T_all{end}(1:3,1:3);
            eul = rotm2eul(R);
            phi = eul(1);
            theta = eul(2);
            psi = eul(3);

            % T for ZYZ
            % T = [0 -sin(phi) cos(phi)*sin(theta);
            %      0 cos(phi) sin(phi)*sin(theta);
            %      1 0 cos(theta)];

            % T for ZYX
            T = [0 -sin(phi)  cos(phi)*cos(theta);
                0  cos(phi)  sin(phi)*cos(theta);
                1  0        -sin(theta)];
            % T = [1 0 -sin(theta);
            %      0 cos(phi) sin(phi)*cos(theta);
            %      0 -sin(phi) cos(phi)*cos(theta)];

            % Ti = inv(T);
            TA = [eye(3) zeros(3); zeros(3) T];

            % Converting the geometric jacobian to the correct form
            Jtemp = zeros(size(obj.J));
            Jtemp(1:3,:) = obj.J(4:6,:);
            Jtemp(4:6,:) = obj.J(1:3,:);

            obj.JA = TA\Jtemp;
            % obj.JA = pinv(TA)*Jtemp;

        end
       
        function obj = computeAnalyticalHessian(obj)
            % H_{ijk} = ∂²x_i/∂q_j∂q_k = ∂(JA_ij)/∂q_k
            
            N = length(obj.q);  % 7 joints
            obj.H = zeros(6, N, N);  % Initialize Hessian tensor
            
            epsilon = 1e-6;
            q_orig = obj.q;
            
            % Store original Jacobian
            obj = obj.computeJacobians();
            obj = obj.computeAnalyticalJacobian();
            JA_orig = obj.JA;
            
            % For each joint variable k
            for k = 1:N
                % Perturb joint k and compute Jacobian
                q_plus = q_orig;
                q_plus(k) = q_plus(k) + epsilon;
                obj = obj.FKM(q_plus);
                obj = obj.computeJacobians();
                obj = obj.computeAnalyticalJacobian();
                JA_plus = obj.JA;
                
                q_minus = q_orig;
                q_minus(k) = q_minus(k) - epsilon;
                obj = obj.FKM(q_minus);
                obj = obj.computeJacobians();
                obj = obj.computeAnalyticalJacobian();
                JA_minus = obj.JA;
                
                % Compute derivative of Jacobian wrt q_k
                dJA_dqk = (JA_plus - JA_minus) / (2 * epsilon);
                
                % Store in Hessian tensor
                obj.H(:, :, k) = dJA_dqk;
            end
            
            % Restore original configuration
            obj = obj.FKM(q_orig);
            obj = obj.computeJacobians();
            obj = obj.computeAnalyticalJacobian();
        end

        function ddx = computePoseAcceleration(obj, qdot, qddot)
            % Computes pose acceleration using analytical Jacobian and Hessian
            % ddx = JA * qddot + Σ_j Σ_k H(:,j,k) * qdot(j) * qdot(k)
            
            N = length(obj.q);
            
            % First term: JA * qddot
            ddx_linear = obj.JA * qddot;
            
            % Second term: quadratic velocity term using Hessian
            ddx_quadratic = zeros(6, 1);
            for i = 1:6
                for j = 1:N
                    for k = 1:N
                        ddx_quadratic(i) = ddx_quadratic(i) + obj.H(i, j, k) * qdot(j) * qdot(k);
                    end
                end
            end
            
            ddx = ddx_linear + ddx_quadratic;
        end

        function cost = IKMCost(obj, sigma, sigma0, qdot0)
            q = sigma(1:7);
            s = sigma(8:end);
            % sp = sigma(1:6);
            % sv = sigma(7:12);
            q0 = sigma0(1:7);
            W = obj.W;
            rhoVec = obj.rhoVec;

            qdot = (q - q0)/obj.dt;
            V = obj.V;
        
            cost = (q-q0).'*W*(q-q0) + (qdot-qdot0).'*V*(qdot-qdot0) +  rhoVec.'*s;
        end

        function [cineq, ceq] = IKMConstraints(obj, sigma, sigma0, pose_des, poseVel_des, firstTime)
            q = sigma(1:7);
            s = sigma(8:end);
            sp = s(1:6);
            sv = s(7:12);
            
            % Extracting the previous joint configuration, for joint
            % velocity calculations
            q0 = sigma0(1:7);
            qdot = (q - q0)/obj.dt;

            obj = obj.FKM(q);
            p = obj.T_all{end}(1:3,4);
            R = obj.T_all{end}(1:3,1:3);

            eul = rotm2eul(R);
            
            pose = [p;eul.'];
            % pose_des = [p_des;eul_des];
        
            % Equality constraints
            ceq = [];
        
            % Pose inequality constraints
            c_upper = pose - sp - pose_des;
            c_lower = -pose - sp + pose_des;

        
            % Inequality constraint Jacobian
            phi = eul(1);
            theta = eul(2);
            
            % Computing the analytical jacobian at the current
            % configuration
            obj = obj.computeJacobians();
            obj = obj.computeAnalyticalJacobian();
            JA = obj.JA;
            % JA(isnan(JA)) = 0;

            cv_upper = JA*qdot - sv - poseVel_des;
            cv_lower = -JA*qdot - sv + poseVel_des;

            if ~firstTime
                cineq = [c_upper;c_lower;cv_upper;cv_lower];
            else
                cineq = [c_upper;c_lower];
            end

            % cineq = [c_upper;c_lower];

        end
        
        %% Dynamics

        function obj = computeMassMatrix(obj)
            % Computes the mass matrix
            % M(q) = Σᵢ [mᵢJᵥᵢᵀJᵥᵢ + JωᵢᵀRᵢIᵢRᵢᵀJωᵢ]

            N = length(obj.q);  % 7 joints
            L = length(obj.Ts); % 8 links including end-effector

            % Initialize mass matrix
            obj.M = zeros(N, N);

            % For each link (including end-effector), compute its contribution
            for i = 1:L

                % Get mass and inertia tensor for link i
                mi = obj.mArr{i};
                Ii = obj.I{i};  % Inertia tensor about COM

                % Compute transformation to link i's COM
                T_COM_i = obj.computeCOMTransform(i);

                % Extract rotation matrix and COM position
                R_i = T_COM_i(1:3, 1:3);  % Rotation from base to link i COM frame
                r_COM_i = T_COM_i(1:3, 4); % COM position in base frame

                % Compute Jacobians for link i's COM
                [Jv_i, Jw_i] = obj.computeLinkJacobians(i, r_COM_i);

                % Add contribution to mass matrix
                % Linear contribution: mᵢJᵥᵢᵀJᵥᵢ
                obj.M = obj.M + mi * (Jv_i' * Jv_i);

                % Angular contribution: JωᵢᵀRᵢIᵢRᵢᵀJωᵢ
                % Transform inertia tensor to base frame: R_i * I_i * R_i'
                I_base = R_i * Ii * R_i';
                obj.M = obj.M + Jw_i' * I_base * Jw_i;
            end
        end

        % function T_COM = computeCOMTransform(obj, link_idx)
        %     % Computes transformation matrix from base frame to COM of link_idx
        %     % COM is defined in the frame of joint (link_idx-1) per Kinova manual
        % 
        %     N = length(obj.q);
        % 
        %     if link_idx == 1
        %         % Base link COM - defined in base frame
        %         T_COM = [eye(3) obj.r_COM{1}; 0 0 0 1];
        % 
        %     elseif link_idx <= N
        %         % Joint link COM: accumulate transformations up to joint (link_idx-1)
        %         T_accum = eye(4);
        %         for j = 1:(link_idx-1)
        %             R_j = [cos(obj.q(j)) -sin(obj.q(j)) 0 0;
        %                    sin(obj.q(j))  cos(obj.q(j)) 0 0;
        %                    0               0            1 0;
        %                    0               0            0 1];
        %             A_j = obj.Ts{j} * R_j;
        %             T_accum = T_accum * A_j;
        %         end
        % 
        %         % Transform COM from joint (link_idx-1) frame to base frame
        %         r_COM_local = [obj.r_COM{link_idx}; 1];
        %         r_COM_base = T_accum * r_COM_local;
        %         T_COM = [T_accum(1:3,1:3) r_COM_base(1:3); 0 0 0 1];
        % 
        %     else
        %         % End-effector (link 8): COM defined in Joint 7's frame
        %         % So use T_all{7} (base to joint 7) + COM offset
        %         T_accum = obj.T_all{N};  % Base to Joint 7
        %         r_COM_local = [obj.r_COM{link_idx}; 1];
        %         r_COM_base = T_accum * r_COM_local;
        %         T_COM = [T_accum(1:3,1:3) r_COM_base(1:3); 0 0 0 1];
        %     end
        % end
        % 


        function T_COM = computeCOMTransform(obj, link_idx)
            % Computes transformation matrix from base frame to COM of link_idx

            N = length(obj.q);

            if link_idx == 1
                % Base link COM - just the COM offset from base
                T_COM = [eye(3) obj.r_COM{1}; 0 0 0 1];
            else
                % Add COM offset for this link
                T_accum = obj.T_all{link_idx-1};
                T_COM_offset = [eye(3) obj.r_COM{link_idx}; 0 0 0 1];
                T_COM = T_accum * T_COM_offset;
            end
        end

        function [Jv, Jw] = computeLinkJacobians(obj, link_idx, r_COM)
            % Computes linear (Jv) and angular (Jw) Jacobians for link_idx COM

            N = length(obj.q);
            Jv = zeros(3, N);  % Linear velocity Jacobian
            Jw = zeros(3, N);  % Angular velocity Jacobian

            % Determine which joints affect this link
            if link_idx == 1
                % Base link is not affected by any joint motion
                return;  % Jacobians remain zero
            % elseif link_idx <= N
            %     % Joint link i is affected by joints 1 to (i-1)
            %     affected_joints = 1:(link_idx-1);
            % else
            %     % End-effector is affected by all joints
            %     affected_joints = 1:N;
            % end
            else
                affected_joints = 1:(link_idx-1);
            end

            

            % For each joint that affects this link
            for j = affected_joints

                % Get the axis and origin of joint j
                % [z_j, o_j] = obj.getJointAxisAndOrigin(j);
                T0j = obj.T_all{j};
                z_j = T0j(1:3,3);
                o_j = T0j(1:3,4);


                % Angular velocity contribution (same for all points on the link)
                Jw(:, j) = z_j;

                % Linear velocity contribution
                Jv(:, j) = cross(z_j, r_COM - o_j);
            end
        end

        function [z_axis, origin] = getJointAxisAndOrigin(obj, joint_idx)
            % Returns the axis of rotation and origin for joint_idx in base frame
            T_frame = obj.T_all{joint_idx};
            z_axis = T_frame(1:3,3);
            origin = T_frame(1:3,4);

        end

        function obj = computeCoriolisMatrix(obj, qdot)
            % Computes the Coriolis matrix C(q,qdot) using Christoffel symbols
            % C_ij = Σₖ c_ijk * qdot_k
            % where c_ijk = 1/2 * [∂M_ij/∂q_k + ∂M_ik/∂q_j - ∂M_jk/∂q_i]
            
            
            N = length(obj.q);
            obj.C = zeros(N, N);
            
            % Compute partial derivatives of mass matrix numerically
            dM_dq = obj.computeMassMatrixDerivatives();
            
            % Compute Christoffel symbols and Coriolis matrix
            for i = 1:N
                for j = 1:N
                    for k = 1:N
                        % Christoffel symbol c_ijk
                        c_ijk = 0.5 * (dM_dq(i,j,k) + dM_dq(i,k,j) - dM_dq(j,k,i));
                        
                        % Add contribution to Coriolis matrix
                        obj.C(i,j) = obj.C(i,j) + c_ijk * qdot(k);
                    end
                end
            end
        end
        
        function dM_dq = computeMassMatrixDerivatives(obj)
            % Computes partial derivatives of mass matrix ∂M/∂q numerically
            % Returns 3D array: dM_dq(i,j,k) = ∂M_ij/∂q_k
            
            N = length(obj.q);
            dM_dq = zeros(N, N, N);
            
            % Small perturbation for numerical differentiation
            % epsilon = 1e-6;
            
            
            % Store original configuration and mass matrix
            q_orig = obj.q;
            M_orig = obj.M;
            
            % Compute derivatives for each joint variable
            for k = 1:N
                epsilon = max(1e-6, 1e-4 * abs(obj.q(k))); % Scale with joint angle
                % Forward perturbation
                q_plus = q_orig;
                q_plus(k) = q_plus(k) + epsilon;
                obj = obj.FKM(q_plus);
                obj = obj.computeMassMatrix();
                M_plus = obj.M;
                
                % Backward perturbation  
                q_minus = q_orig;
                q_minus(k) = q_minus(k) - epsilon;
                obj = obj.FKM(q_minus);
                obj = obj.computeMassMatrix();
                M_minus = obj.M;
                
                % Central difference approximation
                dM_dq(:,:,k) = (M_plus - M_minus) / (2 * epsilon);
            end
            
            % Restore original configuration
            obj = obj.FKM(q_orig);
            obj.M = M_orig;
        end

        function obj = computeGravityTorque(obj)
            N = length(obj.q);
            L = length(obj.Ts);

            obj.G = zeros(N, 1);


            % For each joint i
            for i = 1:N
                % Get joint axis and origin in base frame
                [z_i, o_i] = obj.getJointAxisAndOrigin(i);

                % Sum contributions from links that are kinematically affected by joint i
                for j = 1:L  % Skip base link (j=1)
                    % Check if link j is affected by joint i
                    link_affected = (i < j);

                    if ~link_affected
                        continue;
                    end

                    % Get current COM position in base frame
                    T_COM_j = obj.computeCOMTransform(j);
                    r_COM_j = T_COM_j(1:3, 4);

                    % Analytical derivative using twist: dr/dq = z × (r - o)
                    dr_COM_dqi = cross(z_i, r_COM_j - o_i);


                    % Add gravity contribution
                    mj = obj.mArr{j};
                    contribution = -mj * (obj.gravity' * dr_COM_dqi);
                    obj.G(i) = obj.G(i) + contribution;

                end

            end
        end

        % 
        % function obj = computeGravityTorque(obj)
        %     % Computes gravity torque using geometric Jacobians
        %     % G_i = -∑_j m_j * J_v_j^T * g
        %     % where J_v_j is the linear velocity Jacobian for link j's COM
        % 
        %     N = length(obj.q);
        %     L = length(obj.Ts);
        %     obj.G = zeros(N, 1);
        % 
        %     % For each link (skip base link which doesn't move)
        %     for j = 2:L
        %         % Get link mass
        %         m_j = obj.mArr{j};
        % 
        %         % Get COM position in base frame
        %         T_COM = obj.computeCOMTransform(j);
        %         r_COM = T_COM(1:3, 4);
        % 
        %         % Compute Jacobian for this COM
        %         [J_v, J_w] = obj.computeLinkJacobians(j, r_COM);
        %         Jcj = [J_v;J_w];
        % 
        %         % % Determine which joints affect this link
        %         % for i = 1:N
        %         %     % if j <= N
        %         %     %     affects_link = (i < j); 
        %         %     % else
        %         %     %     affects_link = (i <= N); % End-effector affected by all joints
        %         %     % end
        %         %     affects_link = i < j;
        %         % 
        %         %     if affects_link
        %         %         % Get joint i axis and origin
        %         %         [z_i, o_i] = obj.getJointAxisAndOrigin(i);
        %         % 
        %         %         % Linear velocity Jacobian column: J_v = z_i × (r_COM - o_i)
        %         %         J_v(:, i) = cross(z_i, r_COM - o_i);
        %         %     end
        %         % end
        % 
        %         % Add this link's contribution to gravity torque
        %         Fcj = m_j*obj.gravity;
        %         tau_j = [Fcj;zeros(3,1)];
        % 
        %         obj.G = obj.G - (Jcj.')*tau_j;
        %         % obj.G = obj.G - m_j * (J_v' * obj.gravity);
        %     end
        % end
        % 
        % 
        function affected = linkAffectedByJoint(obj, link_idx, joint_idx)
            % Determines if link_idx is affected by joint_idx motion
            
            if link_idx == 1
                % Base link is never affected by joint motion
                affected = false;
            elseif link_idx <= length(obj.q)
                % Joint link i is affected by joints 1 to (i-1)
                affected = (joint_idx < link_idx);
            else
                % End-effector is affected by all joints
                affected = (joint_idx <= length(obj.q));
            end
        end

        %% Control
        function u = inverseDynamicsControl(obj, QD, Q, M, vProduct, G)

            % Update the dynamics based on the joint configuration
            qdot = Q(:,2);
            obj = obj.FKM(Q(:,1));
            obj = obj.computeJacobians();
            obj = obj.computeMassMatrix();
            obj = obj.computeCoriolisMatrix(qdot);
            obj = obj.computeGravityTorque();
            obj = obj.computeAnalyticalJacobian();

            y = obj.NLSF(QD, Q);

            % Apply the control law
            n = obj.C*qdot + obj.G;
            u = obj.M*y + n;

            % % Applying the control law using matlab's parameters
            % n = vProduct + G;
            % u = M*y + n;
            


            obj.G
            G
            
        end

        function y = NLSF(obj, QD, Q)
            % Change to controlling in joint space for now
            KD = obj.KD;
            KP = obj.KP;
            
            qD = QD(:,1);
            qDdot = QD(:,2);
            qDddot = QD(:,3);

            q = Q(:,1);
            qdot = Q(:,2);
            qddot = Q(:,3);

            r = qDddot + KD*qDdot + KP*qD;
            y = -KP*q - KD*qdot + r;

            % E = Qd - Q;
            % e = E(1);
            % edot = E(2);
            % eddot = E(3);

            % y = pinv(myKG3.JA)*(eddot + KD*edot + KP*e - )

        end

        function S = skew(v)
            % Creates skew-symmetric matrix from 3D vector
            S = [0    -v(3)  v(2);
                 v(3)  0    -v(1);
                 -v(2) v(1)  0   ];
        end

    end 

    methods (Static)

        function [Ts, mArr, I, rC, KD, KP] = preCompute()
            % Creating the coordinate transformations
            Ts = {};

            T01 = [1 0 0 0;
                   0 -1 0 0;
                   0 0 -1 0.1564;
                   0 0 0 1];
            Ts{1} = T01;

            T12 = [1 0 0 0;
                   0 0 -1 0.0054;
                   0 1 0 -0.1284;
                   0 0 0 1];
            Ts{2} = T12;

            T23 = [1 0 0 0;
                0 0 1 -0.2104;
                0 -1 0 -0.00640;
                0 0 0 1];
            Ts{3} = T23;

            T34 = [1 0 0 0;
                0 0 -1 0.0064;
                0 1 0 -0.2104;
                0 0 0 1];
            Ts{4} = T34;

            T45 = [1 0 0 0;
                0 0 1 -0.2084;
                0 -1 0 -0.0064;
                0 0 0 1];
            Ts{5} = T45;

            T56 = [1 0 0 0;
                0 0 -1 0;
                0 1 0 -0.1059;
                0 0 0 1];
            Ts{6} = T56;

            T67 = [1 0 0 0;
                0 0 1 -0.1059;
                0 -1 0 0;
                0 0 0 1];
            Ts{7} = T67;

            T78 = [1 0 0 0;
                   0 -1 0 0;
                   0 0 -1 -0.0615;
                   0 0 0 1];
            Ts{8} = T78;

            %% Initialising properties required for dynamics simulation
            % Using Kinova Gen3 manual parameters
            % Structure: {Base, Link1, Link2, Link3, Link4, Link5, Link6, Link7+EndEffector}
            % mArr = {1.667, 1.3773, 1.1636, 1.1636, 0.9302, 0.6781, 0.6781, 0.500}; % with vision module
            % mArr = {0, 1.3773, 1.1636, 1.1636, 0.9302, 0.6781, 0.6781, 0.500}; % with vision module
            % 
            % % Defining the centre of mass position vectors
            % % Note: Base COM is in base frame, other COMs are in preceding joint frame
            % rC = {};
            % % rC{1} = [-0.000648; -0.000166; 0.084487];   % Base (Table 75)
            % rC{1} = [0; 0; 0];   % Base (Table 75)
            % rC{2} = [-2.3000e-05; -0.0104; -0.0734]; % Link 1 (Table 76)  
            % % rC{2} = [2.3000e-05; 0.0104; 0.0734]; % Link 1 (Table 76)  
            % rC{3} = [-4.4000e-05; -0.0996; -0.0133]; % Link 2 (Table 77)
            % rC{4} = [-4.4000e-05; -0.0066; -0.1179]; % Link 3 (Table 78)
            % rC{5} = [-1.8000e-05; -0.0755; -0.0150]; % Link 4 (Table 79)
            % rC{6} = [1.0000e-06; -0.0094; -0.0639];  % Link 5 (Table 80)
            % rC{7} = [1.0000e-06; -0.0455; -0.0097];  % Link 6 (Table 81)
            % rC{8} = [2.8100e-04; 0.0114; -0.0298]; % Interface module with vision (Table 83)
            % 
            % % Defining the inertia tensors for each link (exact manual values)
            % % Manual format: [Ixx, Ixy, Ixz, Iyy, Iyz, Izz] -> Convert to 3x3 matrix
            % I_vecs = [
            %     % Base (Table 75)
            %     0.004622 0.000009 0.000060 0.004495 0.000009 0.002079;
            %     % Link 1 (Table 76) 
            %     0.004570 0.000001 0.000002 0.004831 0.000448 0.001409;
            %     % Link 2 (Table 77)
            %     0.011088 0.000005 0.000000 0.001072 -0.000691 0.011255;
            %     % Link 3 (Table 78)
            %     0.010932 0.000000 -0.000007 0.011127 0.000606 0.001043;
            %     % Link 4 (Table 79)
            %     0.008147 -0.000001 0.000000 0.000631 -0.000500 0.008316;
            %     % Link 5 (Table 80)
            %     0.001596 0.000000 0.000000 0.001607 0.000256 0.000399;
            %     % Link 6 (Table 81)
            %     0.001641 0.000000 0.000000 0.000410 -0.000278 0.001641;
            %     % Interface module with vision (Table 83)
            %     0.000587 0.000003 0.000003 0.000369 0.000118 0.000609
            % ];

            mArr = {1.697, 1.377, 1.1636, 1.1636, 0.930, 0.678, 0.678, 0.500}; % with vision module
            % mArr = {1.697, 1.377, 1.1636, 1.1636, 0.930, 0.678, 0.678, 1.425}; % includes gripper module

            % Defining the centre of mass position vectors
            % Note: Base COM is in base frame, other COMs are in preceding joint frame
            rC = {};
            % rC{1} = [0; 0; 0];   % Base (Table 75)
            % rC{2} = [-0.000023; -0.010364; -0.073360]; % Link 1 (Table 76)  
            % rC{3} = [-0.000044; -0.099580; -0.013278]; % Link 2 (Table 77)
            % rC{4} = [-0.000044; -0.006641; -0.117892]; % Link 3 (Table 78)
            % rC{5} = [-0.000018; -0.075478; -0.015006]; % Link 4 (Table 79)
            % rC{6} = [0.000001; -0.009432; -0.063883];  % Link 5 (Table 80)
            % rC{7} = [0.000001; -0.045483; -0.009650];  % Link 6 (Table 81)
            % rC{8} = [0.000281; 0.011402; -0.029798]; % Interface module with vision (Table 83)
            % rC{8} = [-0.000281; -0.011402; -0.077098]; % Interface module with vision (Table 83)

            rC{1} = [0; 0; 0];   % Base (Table 75)
            rC{2} = [-0.000023; -0.010364; -0.073360]; % Link 1 (Table 76)  
            rC{3} = [-0.000044; -0.099580; 0.013278]; % Link 2 (Table 77)
            rC{4} = [-0.000044; -0.006641; -0.117892]; % Link 3 (Table 78)
            rC{5} = [-0.000018; -0.075478; -0.015006]; % Link 4 (Table 79)
            rC{6} = [0.000001; -0.009032; -0.063883];  % Link 5 (Table 80)
            rC{7} = [0.000001; -0.045483; -0.009650];  % Link 6 (Table 81)
            rC{8} = [0.000281; 0.011402; -0.029798]; % Interface module with vision (Table 83)

            % Defining the inertia tensors for each link (exact manual values)
            % Manual format: [Ixx, Ixy, Ixz, Iyy, Iyz, Izz] -> Convert to 3x3 matrix
            % I_vecs = [
            %     % Base (Table 75)
            %     0.004622 0.000009 0.000060 0.004495 0.000009 0.002079;
            %     % Link 1 (Table 76) 
            %     0.004570 0.000001 0.000002 0.004831 0.000448 0.001409;
            %     % Link 2 (Table 77)
            %     0.011088 0.000005 0.000000 0.001072 -0.000691 0.011255;
            %     % Link 3 (Table 78)
            %     0.010932 0.000000 -0.000007 0.011127 0.000606 0.001043;
            %     % Link 4 (Table 79)
            %     0.008147 -0.000001 0.000000 0.000631 -0.000500 0.008316;
            %     % Link 5 (Table 80)
            %     0.001596 0.000000 0.000000 0.001607 0.000256 0.000399;
            %     % Link 6 (Table 81)
            %     0.001641 0.000000 0.000000 0.000410 -0.000278 0.001641;
            %     % Interface module with vision (Table 83)
            %     0.000587 0.000003 0.000003 0.000369 0.000118 0.000609
            %     % 0.000692 0.000003 0.000003 0.000542 0.000118 0.0001599  %gripper module
            % ];

            I_vecs = [
                % Base (Table 75)
                0.004622 0.000009 0.000060 0.004495 0.000009 0.002079;
                % Link 1 (Table 76) 
                0.0121 0.0122 0.0016 -5.9917e-04 -3.2389e-07 6.7169e-07;
                % Link 2 (Table 77)
                0.0228 0.0013 0.0228 -0.0022 -6.7981e-07 -9.8337e-08;
                % Link 3 (Table 78)
                0.0272 0.0273 0.0011 -3.0501e-04 -1.3036e-05 -3.4001e-07;
                % Link 4 (Table 79)
                0.0137 8.4046e-04 0.0136 -0.0016 -2.5125e-07 -2.2638e-06;
                % Link 5 (Table 80)
                0.0044 0.0044 4.5933e-04 -1.5259e-04 4.3319e-08 6.3958e-09;
                % Link 6 (Table 81)
                0.0031 4.7315e-04 0.0030 -5.7563e-04 6.5437e-09 3.0842e-08;
                % Interface module with vision (Table 83)
                0.0011 8.1300e-04 6.7404e-04 5.1878e-05 7.1866e-06 1.3980e-06
                % 0.000692 0.000003 0.000003 0.000542 0.000118 0.0001599  %gripper module
            ];

                        % Convert inertia vectors to 3x3 matrices
            I = {};
            n = length(I_vecs);
            for i=1:n
                % Extract inertia vector components
                Ii = I_vecs(i,:);
                Ixx = Ii(1);
                Iyy = Ii(2);  % Manual: Ixy is 2nd element
                Izz = Ii(3);  % Manual: Ixz is 3rd element  
                Iyz = Ii(4);  % Manual: Iyy is 4th element
                Ixz = Ii(5);  % Manual: Iyz is 5th element
                Ixy = Ii(6);  % Manual: Izz is 6th element
                % Ixy = 0;
                % Ixz = 0;
                % Iyz = 0;

                % Form symmetric inertia tensor
                I{i} = [Ixx Ixy Ixz;
                        Ixy Iyy Iyz;
                        Ixz Iyz Izz];
            end
            

            % % Convert inertia vectors to 3x3 matrices
            % I = {};
            % n = length(I_vecs);
            % for i=1:n
            %     % Extract inertia vector components
            %     Ii = I_vecs(i,:);
            %     Ixx = Ii(1);
            %     Ixy = Ii(2);  % Manual: Ixy is 2nd element
            %     Ixz = Ii(3);  % Manual: Ixz is 3rd element  
            %     Iyy = Ii(4);  % Manual: Iyy is 4th element
            %     Iyz = Ii(5);  % Manual: Iyz is 5th element
            %     Izz = Ii(6);  % Manual: Izz is 6th element
            %     % Ixy = 0;
            %     % Ixz = 0;
            %     % Iyz = 0;
            % 
            %     % Form symmetric inertia tensor
            %     I{i} = [Ixx Ixy Ixz;
            %             Ixy Iyy Iyz;
            %             Ixz Iyz Izz];
            % end

            % KP = 8*eye(7,7);
            L = [8,8,8,8,8,8,8];
            % L = 8*ones(1,7);
            KP = diag(L);
            KD = diag(1.5*L);
            % KD = 2*8*eye(7,7);


        end


        % function [Ts, mArr, I, rC] = preCompute()
        %     % Creating the coordinate transformations
        %      Ts = {};
        %      T01 = [1 0 0 0;
        %      0 -1 0 0;
        %      0 0 -1 0.1564;
        %      0 0 0 1];
        %      Ts{1} = T01;
        %      T12 = [1 0 0 0;
        %      0 0 -1 0.0054;
        %      0 1 0 -0.1284;
        %      0 0 0 1];
        %      Ts{2} = T12;
        %      T23 = [1 0 0 0;
        %      0 0 1 -0.2104;
        %      0 -1 0 -0.00640;
        %      0 0 0 1];
        %      Ts{3} = T23;
        %      T34 = [1 0 0 0;
        %      0 0 -1 0.0064;
        %      0 1 0 -0.2104;
        %      0 0 0 1];
        %      Ts{4} = T34;
        %      T45 = [1 0 0 0;
        %      0 0 1 -0.2084;
        %      0 -1 0 -0.0064;
        %      0 0 0 1];
        %      Ts{5} = T45;
        %      T56 = [1 0 0 0;
        %      0 0 -1 0;
        %      0 1 0 -0.1059;
        %      0 0 0 1];
        %      Ts{6} = T56;
        %      T67 = [1 0 0 0;
        %      0 0 1 -0.1059;
        %      0 -1 0 0;
        %      0 0 0 1];
        %      Ts{7} = T67;
        %      T78 = [1 0 0 0;
        %      0 -1 0 0;
        %      0 0 -1 -0.0615;
        %      0 0 0 1];
        %      Ts{8} = T78;
        % 
        %     % Load toolbox robot
        %     gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2);
        % 
        %     % Extract ALL parameters from toolbox for consistency
        %     mArr = {1.667}; % Base mass only from manual
        %     I = {[0.004622 0.000009 0.000060;
        %           0.000009 0.004495 0.000009; 
        %           0.000060 0.000009 0.002079]}; % Base inertia from manual
        %     rC = {[-0.000648; -0.000166; 0.084487]}; % Base COM from manual
        % 
        %     for i = 1:length(gen3.Bodies)
        %         % Extract mass
        %         mArr{end+1} = gen3.Bodies{i}.Mass;
        % 
        %         % Extract and convert inertia
        %         I_vec = gen3.Bodies{i}.Inertia;
        %         I_matrix = [I_vec(1) I_vec(6) I_vec(5);
        %                    I_vec(6) I_vec(2) I_vec(4);
        %                    I_vec(5) I_vec(4) I_vec(3)];
        %         I{end+1} = I_matrix;
        % 
        %         % Extract COM (in joint frame)
        %         rC{end+1} = gen3.Bodies{i}.CenterOfMass.';
        %     end
        % end
        % 

    end
end