classdef KG3
    % TODO: ADD CLASS DESCRIPTION

    properties
        %% FKM properties
        Ts
        T_all % all transformation matricies for the KG3 T0i (referred to base frame).
        Aij_all % local transformations between matricies

        dTdq_all 
        % dAdqi_all % cells which store the derivatives of the homogenous transformation matricies
        % dAdq_all  % cells which store the tensors for the derivatives of the homogenous transformation matricies 

        pose  % pose of the KG3.

        T_COM % cells which store the transformations between the COMs and the preeceding link frames.
        r_COM

        AC_all
        % dACidqi_all % cells which store the derivatives of the COM homogeneous transformation matricies
        dACdq_all   % cells which store the tensors for the derivatives of the COM homogenous transformation matricies 

        %% Differential Kinematics Properties
        J % geometric jacobian.
        J_COM % cells which store the COM geometric Jacobians.
        dJdq_COM
        
        %% Dynamics Properties
        mArr %kg, array each stores the mass of each link in the KG3.
        I    % cells which store the inertia tensors of each link referred to the COM.
        q     % current joint configuration
        M     % mass matrix 
        C     % Coriolis matrix
    end

    methods
        function obj = KG3()
            %KG3 Construct an instance of this class
            % Precomputes the homogenous transformations between each frame
            % of the KG3 
            [obj.Ts, obj.mArr, obj.T_COM, obj.I, obj.r_COM] = KG3.preCompute();
        end
       
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
            % obj.dAdqi_all{N+1} = obj.dAdqi_all{N};


            % % Compute full transforms to each joint
            % obj.T_all = cell(1,N);
            % obj.dAdqi_all = cell(1,N);
            % T_base = eye(4);
            % for j = 1:N
            %     A = obj.Ts{j} * Rs{j};
            %     T_base = T_base * A;
            %     obj.T_all{j} = T_base;
            % 
            %     % Computing the derivative of each Ai-1_i transformation
            %     obj.dAdqi_all{j} = KG3.hatSE3(xi)*A;   
            % end

            %% Computing the COM transforms
            
            % Creating an empty cells to store the transforms
            obj.AC_all = cell(1,L);
            obj.AC_all{1} = [eye(3) obj.r_COM{1};
                             zeros(1,3) 1];
            for i=2:L
                % Notation:
                % j = i-1
                % Ci = ith COM
                % rjCi = position of the ith COM referred to the ith i-1
                % joint frame

                rjCi = obj.r_COM{i};
                % Extracting the transform for frame i-1 referred to the
                % base. N.B. T_all{1} = A01
                A0j = obj.T_all{i-1};

                % Forming the transform between the joint reference frame
                % and the ith COM
                AjCi = [eye(3) rjCi;
                        zeros(1,3) 1];
                obj.AC_all{i} = A0j*AjCi;

            end

       

            % Computing the full tensors for the transformation derivatives
            % obj.dAdq_all = KG3.computeTensor(obj.dAdqi_all, obj.Aij_all);
            % obj.dACdq_all = KG3.computeTensor(obj.dACidqi_all, obj.AC_all);

        end
        
        function obj = JGEO_COM(obj)
            % Computes the COM geometric Jacobians
            N = 7;  % number of joints
            L = 8;  % number of links
            obj.J_COM = cell(1,L);

            for i = 1:L
                Jci = zeros(6,N); % 6x7 Jacobian for link i
                pCi = obj.AC_all{i}(1:3,4); % COM position in base frame

                if i > 1
                    % Only joints affecting this link
                    for j = 1:min(i-1,N)
                        p_j = obj.T_all{j}(1:3,4);       % joint origin
                        z_j = obj.T_all{j}(1:3,1:3) * [0;0;1]; % joint axis in base
                        Jv = cross(z_j, pCi - p_j);      % linear velocity part
                        Jw = z_j;                         % angular velocity part
                        Jci(:,j) = [Jv; Jw];
                    end
                end
                % For i = 1, Jci stays all zeros
                obj.J_COM{i} = Jci;
            end
        end

        function obj = JGEO(obj)
            % Computes the geometric Jacobian of the KG3 for the end-effector
            % using z_{i-1} = axis of joint i expressed in base frame.
        
            pe = obj.T_all{end}(1:3,4);           % End-effector position
            N  = numel(obj.T_all) - 1;            % number of joints
            obj.J = zeros(6, N);
        
            for i = 1:N
                if i == 1
                    R_prev = eye(3);              % Base frame for joint 1
                    p_prev = [0;0;0];
                else
                    R_prev = obj.T_all{i-1}(1:3,1:3);  % rotation base -> joint i-1
                    p_prev = obj.T_all{i-1}(1:3,4);    % position of joint i-1
                end
        
                z = R_prev * [0;0;1];              % axis of joint i in base frame
                p_joint = p_prev;                  % origin of joint i in base frame
        
                Jp = cross(z, pe - p_joint);       % linear velocity part
                Jo = z;                             % angular velocity part
        
                obj.J(:,i) = [Jp; Jo];
            end
        end

        function obj = HESSIAN_GEO_COM(obj)
            % Computes the geometric Hessian for the COM of each link
            %
            % Output:
            %   obj.dJdq_COM : cell array of 6xNxN tensors (Hessians) for each COM

            % Compute the transformation derivatives
            obj = obj.compute_dT_FD(obj.q);
            obj = obj.compute_dT_COM_FD(obj.q);
        
            L = length(obj.AC_all);   % number of links
            % N = length(obj.T_all);    % number of joints
            N = 7;
        
            % Precompute COM position derivatives
            drCdq = cell(1, L);
            for i = 1:L
                % dACdq_all is 4x4xN tensor for each link
                dA0Cidq = obj.dACdq_all{i};   % 4x4xN
                drCdq{i} = squeeze(dA0Cidq(1:3,4,:)); % 3xN
            end
        
            % Precompute joint z-axis and position derivatives
            dzdq = cell(1, L);
            dpdq = cell(1, L);
            dzdq{1} = zeros(3, 7);
            dpdq{1} = zeros(3, 7);
        
            for i = 2:L
                dA0idq = obj.dTdq_all{i};   % 4x4xN
                dzdq{i} = squeeze(dA0idq(1:3,3,:));  % angular derivatives
                dpdq{i} = squeeze(dA0idq(1:3,4,:));  % linear derivatives
            end
        
            % Allocate storage for Hessians
            dJ_COM = cell(1, L);
            dJ_COM{1} = zeros(6, N, N);
        
            for i = 1:L
                rCi = obj.AC_all{i}(1:3,4);  % COM position in base frame
                drCidq = drCdq{i};           % 3xN derivative of COM wrt q
        
                dJci = zeros(6, N, N);       % 6xNxN tensor for Hessian
        
                if i > 1
                    for j = 1:min(i-1, N)   % joints affecting this link
                        p_j = obj.T_all{j}(1:3,4);          % joint origin
                        z_j = obj.T_all{j}(1:3,1:3) * [0;0;1]; % joint axis
        
                        dpjdq = dpdq{j};
                        dzjdq = dzdq{j};
        
                        for k = 1:7
                            % Linear part of Hessian
                            dJv = cross(dzjdq(:,k), rCi - p_j) + ...
                                  cross(z_j, drCidq(:,k) - dpjdq(:,k));
        
                            % Angular part of Hessian
                            dJw = dzjdq(:,k);
        
                            % Store in tensor
                            dJci(:, j, k) = [dJv; dJw];
                        end
                    end
                end
        
                dJ_COM{i} = dJci;
            end
        
            obj.dJdq_COM = dJ_COM;
        end

        function obj = compute_dT_FD(obj, q, eps_fd)
            % Compute derivatives of T_all w.r.t. q using finite differences
            
            if nargin < 3
                eps_fd = 1e-8;
            end
            
            L = numel(obj.T_all);   % number of links
            n = numel(q);           % number of joints
            
            % Initialize derivative tensors
            obj.dTdq_all = cell(L, 1);
            for i = 1:L
                obj.dTdq_all{i} = zeros(4, 4, n);
            end
            
            for k = 1:n
                % Perturb joint k
                qp = q; qm = q;
                qp(k) = qp(k) + eps_fd;
                qm(k) = qm(k) - eps_fd;
                
                % Create temporary objects to avoid overwriting
                obj_temp_p = obj.copyKG3(); 
                obj_temp_m = obj.copyKG3();
                
                % Create new instances and compute FKM
                obj_temp_p = obj_temp_p.FKM(qp);
                obj_temp_m = obj_temp_m.FKM(qm);
                
                Tp_all = obj_temp_p.T_all;
                Tm_all = obj_temp_m.T_all;
                
                for i = 1:L
                    % Finite difference approximation
                    obj.dTdq_all{i}(:,:,k) = (Tp_all{i} - Tm_all{i}) / (2*eps_fd);
                end
            end
        end
        
        function obj = compute_dT_COM_FD(obj, q, eps_fd)
            % Compute derivatives of COM transforms w.r.t. q using finite differences
            
            if nargin < 3
                eps_fd = 1e-8;
            end
            
            % Ensure AC_all is computed for current q
            obj = obj.FKM(q);
            
            L = numel(obj.AC_all);   % number of COM links
            n = numel(q);            % number of joints
            
            % Initialize derivative tensors
            obj.dACdq_all = cell(L, 1);
            for i = 1:L
                obj.dACdq_all{i} = zeros(4, 4, n);
            end
            
            for k = 1:n
                % Perturb joint k
                qp = q; qm = q;
                qp(k) = qp(k) + eps_fd;
                qm(k) = qm(k) - eps_fd;
                
                % Create temporary objects
                obj_temp_p = obj.copyKG3();  % Create independent copies
                obj_temp_m = obj.copyKG3();
                
                % Compute forward kinematics at perturbed configurations
                obj_temp_p = obj_temp_p.FKM(qp);
                obj_temp_m = obj_temp_m.FKM(qm);
                
                Tp_all = obj_temp_p.AC_all;
                Tm_all = obj_temp_m.AC_all;
                
                for i = 1:L
                    % Finite difference approximation
                    obj.dACdq_all{i}(:,:,k) = (Tp_all{i} - Tm_all{i}) / (2*eps_fd);
                end
            end
        end
        
        function obj_copy = copyKG3(obj)
            % Create a deep copy of the KG3 object
            obj_copy = KG3();

            % Copy all relevant properties
            obj_copy.q = obj.q;
            obj_copy.Ts = obj.Ts;
            obj_copy.T_all = obj.T_all;
            obj_copy.Aij_all = obj.Aij_all;
            obj_copy.AC_all = obj.AC_all;
            obj_copy.T_COM = obj.T_COM;
            obj_copy.r_COM = obj.r_COM;
            obj_copy.mArr = obj.mArr;
            obj_copy.I = obj.I;
            obj_copy.J = obj.J;
            obj_copy.J_COM = obj.J_COM;

            % Initialize derivative tensors
            if ~isempty(obj.dTdq_all)
                obj_copy.dTdq_all = obj.dTdq_all;
            end
            if ~isempty(obj.dACdq_all)
                obj_copy.dACdq_all = obj.dACdq_all;
            end
            if ~isempty(obj.dJdq_COM)
                obj_copy.dJdq_COM = obj.dJdq_COM;
            end
        end

        
        %% Computes the mass matrix of the KG3 in its current joint configuration
        function obj = MassMatrix(obj, q)
            N = length(q);  % 7 joints
            L = length(obj.Ts); % 8 links including end-effector
            
            % Compute the forward kinematics to update the transformation
            % matricies
            obj = FKM(obj,q);
            % Use the transformation matricies to update the COM Jacobians
            obj = JGEO_COM(obj);
            
            % Compute the elements of the mass matrix
            obj.M = zeros(N,N);
            for i = 1:L
                mi = obj.mArr{i};                  % mass
                ICi = obj.I{i};                    % inertia in link frame
                R0i = obj.AC_all{i}(1:3,1:3);      % rotation to base frame
                Jci = obj.J_COM{i};                 % 6x7 Jacobian
                Si = blkdiag(mi*eye(3), R0i*ICi*R0i.');
                % M_link = Jci' * [mi*eye(3) zeros(3); zeros(3) R0i*ICi*R0i'] * Jci;
                M_link = Jci.' * Si * Jci;
                obj.M = obj.M + M_link;
            end

        end

        function obj = CoriolisMatrix(obj, q, dqdt)
            % Computes the coriolis matrix of the KG3
            
            % Update forward kinematics and derivatives
            obj = obj.FKM(q);
            obj = obj.JGEO_COM();
            
            % Initialize derivative tensors if not already computed
            if isempty(obj.dTdq_all)
                obj.dTdq_all = cell(1, length(obj.T_all));
                for i = 1:length(obj.T_all)
                    obj.dTdq_all{i} = zeros(4, 4, 7);
                end
            end
            
            if isempty(obj.dACdq_all)
                obj.dACdq_all = cell(1, length(obj.AC_all));
                for i = 1:length(obj.AC_all)
                    obj.dACdq_all{i} = zeros(4, 4, 7);
                end
            end
            
            % Compute transformation derivatives using finite differences
            obj = obj.compute_dT_FD(q);
            obj = obj.compute_dT_COM_FD(q);
            
            % Compute Jacobian derivatives (Hessian)
            obj = obj.HESSIAN_GEO_COM();
            
            % Extract rotation matrices and their derivatives
            N = length(obj.T_all);
            R_all = cell(1, N);
            dR0dq_all = cell(1, N);
            for i = 1:N
                R_all{i} = obj.T_all{i}(1:3, 1:3);
                if ~isempty(obj.dTdq_all{i})
                    dR0dq_all{i} = obj.dTdq_all{i}(1:3, 1:3, :);
                else
                    dR0dq_all{i} = zeros(3, 3, 7);
                end
            end
            
            % Computing the mass matrix derivatives
            L = length(obj.AC_all);
            dMdq = zeros(7, 7, 7);
            
            for k = 1:7
                for i = 1:L
                    dJdq_COMi = obj.dJdq_COM{i}; % Should be 6x7x7
                    mi = obj.mArr{i};
                    
                    % Handle rotation matrix derivatives safely
                    if i <= length(R_all) && i <= length(dR0dq_all)
                        R0i = R_all{i};
                        dR0idq = dR0dq_all{i};
                    else
                        % For end-effector or if missing, use identity
                        R0i = eye(3);
                        dR0idq = zeros(3, 3, 7);
                    end
                    
                    Icii = obj.I{i};
                    J_COMi = obj.J_COM{i}; % Should be 6x7
                    
                    % Check dimensions
                    if size(dJdq_COMi, 2) ~= 7 || size(dJdq_COMi, 3) ~= 7
                        warning('Dimension mismatch in dJdq_COM{%d}: expected 6x7x7, got %s', ...
                            i, mat2str(size(dJdq_COMi)));
                        continue;
                    end
                    
                    % Compute inertia matrices
                    Si = blkdiag(mi*eye(3), R0i*Icii*R0i.');
                    dSi = blkdiag(zeros(3), ...
                        dR0idq(:,:,k)*Icii*R0i.' + R0i*Icii*dR0idq(:,:,k).');
                    
                    % Add contribution to mass matrix derivative
                    dMdq(:,:,k) = dMdq(:,:,k) + ...
                        dJdq_COMi(:,:,k).' * Si * J_COMi + ...
                        J_COMi.' * dSi * J_COMi + ...
                        J_COMi.' * Si * dJdq_COMi(:,:,k);
                end
            end
            
            % Computing the Christoffel symbols
            Gamma = zeros(7, 7, 7);
            for i = 1:7
                for j = 1:7
                    for k = 1:7
                        Gamma(i,j,k) = 0.5*(dMdq(k,j,i) + dMdq(k,i,j) - dMdq(i,j,k));
                    end
                end
            end
            
            % Computing the Coriolis matrix
            obj.C = zeros(7, 7);
            for k = 1:7
                for j = 1:7
                    for i = 1:7
                        obj.C(k,j) = obj.C(k,j) + Gamma(i,j,k)*dqdt(i);
                    end
                end
            end
            
            % Test for skew symmetry (Mdot - 2C should be skew-symmetric)
            Mdot = zeros(7, 7);
            for k = 1:7
                Mdot = Mdot + dMdq(:,:,k) * dqdt(k);
            end
            testMat = Mdot - 2*obj.C;
            skew_error = norm(testMat + testMat.', 'fro');
            
            if skew_error > 1e-10
                warning('Skew symmetry test failed: error = %g (should be ~0)', skew_error);
            else
                fprintf('Skew symmetry test passed: error = %g\n', skew_error);
            end
        end
        
    end

    methods (Static)
        function [Ts, mArr, TC, I, rC] = preCompute()
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
            % Defining the mass of each link   %vision module = 0.500
            % mArr = {1.697, 1.377, 1.1636, 1.1636, 0.930, 0.678, 0.678, 0.364}; % no vision module
            mArr = {1.697, 1.377, 1.1636, 1.1636, 0.930, 0.678, 0.678, 0.500}; % with vision module

            % Defining the centre of mass position vectors (pCi = pi + rCi)
            rC = {};
            rC{1} = [-0.000648;-0.000166;0.084487];
            rC{2} = [-0.000023; -0.010364; -0.073360];
            rC{3} = [-0.000044; -0.099580; -0.013278];
            rC{4} = [-0.000044; -0.006641; -0.117892];
            rC{5} = [-0.000018; -0.075478; -0.015006];
            rC{6} = [0.000001; -0.009432; -0.063883];
            rC{7} = [0.000001; -0.045483; -0.009650];
            % rC{8} = [-0.000093; 0.000132; -0.022905]; % interface module without vision module
            rC{8} = [-0.000281; -0.011402; -0.029798];  % interface module with vision module

            TC = {};

            for i=1:8
                TC{i} = [eye(3,3) rC{i};
                         zeros(1,3) 1];
            end


            
            % Defining the inertia tensors for each link
            % Ii = [Ixx Ixy Ixz Iyy Iyz Izz]
            % I_vecs = [I0;I2;...I7]
            % I_vecs = [0.004622 0.000009 0.000060 0.004495 0.000009 0.002079;
            %           0.004570 0.000001 0.000002 0.004831 0.000448 0.001409;
            %           0.011088 0.000005 0.000000 0.001072 -0.000691 0.011255;
            %           0.010932 0.000000 -0.000007 0.011127 0.000606 0.001043;
            %           0.008147 -0.000001 0.000000 0.000631 -0.000500 0.008316;
            %           0.001596 0.000000 0.000000 0.001607 0.000256 0.000399;
            %           0.001641 0.000000 0.000000 0.000410 -0.000278 0.001641;
            %           0.000214 0.000000 0.000001 0.000223 -0.000002 0.000240];  % without vision module


            I_vecs = [0.004622 0.000009 0.000060 0.004495 0.000009 0.002079;
                      0.004570 0.000001 0.000002 0.004831 0.000448 0.001409;
                      0.011088 0.000005 0.000000 0.001072 -0.000691 0.011255;
                      0.010932 0.000000 -0.000007 0.011127 0.000606 0.001043;
                      0.008147 -0.000001 0.000000 0.000631 -0.000500 0.008316;
                      0.001596 0.000000 0.000000 0.001607 0.000256 0.000399;
                      0.001641 0.000000 0.000000 0.000410 -0.000278 0.001641;
                      0.000587 0.000003 0.000003 0.000369 0.000118 0.000609]; % with vision module
            
            % Looping through each Ivec to generate the inertia tensors
            I = {};
            n = length(I_vecs);
            for i=1:n
                % Extracting the intertia vector and its parameters
                Ii = I_vecs(i,:);
                Ixx = Ii(1);
                Ixy = Ii(2);
                Ixz = Ii(3);
                Iyy = Ii(4);
                Iyz = Ii(5);
                Izz = Ii(6);
                
                % Forming the inertia tensor for link i
                I{i} = [Ixx Ixy Ixz;
                        Ixy Iyy Iyz;
                        Ixz Iyz Izz];
            end

        end

        function G = hatSE3(xi)
            % Computes the generating matrix for SE3 for a given twist vector
            G = [KG3.skew(xi(4:6)) xi(1:3); zeros(1,4)];
        end

        function S = skew(u)
            % Generates a skew symetric matrix from a given vector u
            S = [0 -u(3) u(2);
                u(3) 0 -u(1);
                -u(2) u(1) 0];
        end
    

        function tensors = computeTensor(dAidqi, A_all)


            % Extracting the transformation matrix derivatives wrt qi
            dA01dq1 = dAidqi{1};
            dA12dq2 = dAidqi{2};
            dA23q3 = dAidqi{3};
            dA34q4 = dAidqi{4};
            dA45q5 = dAidqi{5};
            dA56q6 = dAidqi{6};
            dA67q7 = dAidqi{7};

            % Computing the full tensors
            dA01dq = cat(3, dA01dq1, zeros(4, 4), zeros(4, 4), zeros(4, 4), zeros(4, 4), zeros(4, 4), zeros(4, 4));
            dA02dq = cat(3, dA01dq1*A_all{2}, A_all{1}*dA12dq2, zeros(4, 4), zeros(4, 4), zeros(4, 4), zeros(4, 4), zeros(4, 4));
            dA03dq = cat(3, dA01dq1*A_all{2}*A_all{3}, A_all{1}*dA12dq2*A_all{3}, A_all{1}*A_all{2}*dA23q3, zeros(4, 4), zeros(4, 4), zeros(4, 4), zeros(4, 4));
            dA04dq = cat(3, dA01dq1*A_all{2}*A_all{3}*A_all{4}, A_all{1}*dA12dq2*A_all{3}*A_all{4}, A_all{1}*A_all{2}*dA23q3*A_all{4}, A_all{1}*A_all{2}*A_all{3}*dA34q4, zeros(4, 4), zeros(4, 4), zeros(4, 4));
            dA05dq = cat(3, dA01dq1*A_all{2}*A_all{3}*A_all{4}*A_all{5}, A_all{1}*dA12dq2*A_all{3}*A_all{4}*A_all{5}, A_all{1}*A_all{2}*dA23q3*A_all{3}*A_all{4}*A_all{5}, A_all{1}*A_all{2}*A_all{3}*dA34q4*A_all{5}, A_all{1}*A_all{2}*A_all{3}*A_all{4}*dA45q5, zeros(4, 4), zeros(4, 4));
            dA06dq = cat(3, dA01dq1*A_all{2}*A_all{3}*A_all{4}*A_all{5}*A_all{6}, A_all{1}*dA12dq2*A_all{3}*A_all{4}*A_all{5}*A_all{6}, A_all{1}*A_all{2}*dA23q3*A_all{3}*A_all{4}*A_all{5}*A_all{6}, A_all{1}*A_all{2}*A_all{3}*dA34q4*A_all{5}*A_all{6}, A_all{1}*A_all{2}*A_all{3}*A_all{4}*dA45q5*A_all{6}, A_all{1}*A_all{2}*A_all{3}*A_all{4}*A_all{5}*dA56q6, zeros(4, 4));
            dA07dq = cat(3, dA01dq1*A_all{2}*A_all{3}*A_all{4}*A_all{5}*A_all{6}*A_all{7}, A_all{1}*dA12dq2*A_all{3}*A_all{4}*A_all{5}*A_all{6}*A_all{7}, A_all{1}*A_all{2}*dA23q3*A_all{3}*A_all{4}*A_all{5}*A_all{6}*A_all{7}, A_all{1}*A_all{2}*A_all{3}*dA34q4*A_all{5}*A_all{6}*A_all{7}, A_all{1}*A_all{2}*A_all{3}*A_all{4}*dA45q5*A_all{6}*A_all{7}, A_all{1}*A_all{2}*A_all{3}*A_all{4}*A_all{5}*dA56q6*A_all{7}, A_all{1}*A_all{2}*A_all{3}*A_all{4}*A_all{5}*A_all{6}*dA67q7);
            dA08dq = cat(3, dA01dq1*A_all{2}*A_all{3}*A_all{4}*A_all{5}*A_all{6}*A_all{7}*A_all{8}, A_all{1}*dA12dq2*A_all{3}*A_all{4}*A_all{5}*A_all{6}*A_all{7}*A_all{8}, A_all{1}*A_all{2}*dA23q3*A_all{3}*A_all{4}*A_all{5}*A_all{6}*A_all{7}*A_all{8}, A_all{1}*A_all{2}*A_all{3}*dA34q4*A_all{5}*A_all{6}*A_all{7}*A_all{8}, A_all{1}*A_all{2}*A_all{3}*A_all{4}*dA45q5*A_all{6}*A_all{7}*A_all{8}, A_all{1}*A_all{2}*A_all{3}*A_all{4}*A_all{5}*dA56q6*A_all{7}*A_all{8}, A_all{1}*A_all{2}*A_all{3}*A_all{4}*A_all{5}*A_all{6}*dA67q7*A_all{8});
            tensors = {dA01dq,dA02dq,dA03dq,dA04dq,dA05dq,dA06dq,dA07dq,dA08dq};

        end

    end
end