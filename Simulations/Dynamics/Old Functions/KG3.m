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


        % function obj = FKM(obj, q)
        %     % Computes the forward kinematics of the KG3 for a given joint configuration (q)
        %     obj.q = q;
        %     N = length(q);  % 7 joints
        %     L = length(obj.Ts); % 8 links including end-effector
        %     xi = [0;0;0;0;0;1]; % constant local twist for each joint (all joints revolute)
        % 
        %     % Build rotation matrices for each joint (for clarity, optional)
        %     Rs = cell(1,N);
        %     for i = 1:N
        %         Rs{i} = [cos(q(i)) -sin(q(i)) 0 0;
        %                  sin(q(i))  cos(q(i)) 0 0;
        %                  0           0        1 0;
        %                  0           0        0 1];
        %     end
        % 
        %     % --- Forward Kinematics for joints 1:N
        %     T_base = eye(4);
        %     obj.T_all = cell(1,N+1);       % N joints + 1 end-effector
        %     obj.dAdqi_all = cell(1,N+1);   % derivatives
        % 
        %     for j = 1:N
        %         A = obj.Ts{j} * Rs{j};
        %         T_base = T_base * A;
        %         obj.T_all{j} = T_base;
        % 
        %         % derivative of Ai-1_i wrt qi (single joint twist)
        %         obj.dAdqi_all{j} = KG3.hatSE3(xi) * A;   
        %     end
        % 
        %     % --- End-effector (8th link)
        %     T_base = T_base * obj.Ts{N+1};
        %     obj.T_all{N+1} = T_base;
        % 
        %     % Properly populate derivatives for end-effector wrt all joints
        %     obj.dAdqi_all{N+1} = zeros(4,4,N);
        %     for j = 1:N
        %         obj.dAdqi_all{N+1}(:,:,j) = obj.dAdqi_all{j} * obj.Ts{N+1};
        %     end
        % 
        %     % --- COM transforms
        %     obj.AC_all = cell(1,N+1);
        %     obj.dACidqi_all = cell(1,N+1);
        % 
        %     for j = 1:L
        %         if j == 1
        %             % First link: just COM offset relative to base
        %             obj.AC_all{1} = obj.T_COM{1};
        %         else
        %             % Other links: previous joint transform * COM offset
        %             obj.AC_all{j} = obj.T_all{j-1} * obj.T_COM{j};
        %         end
        %     end
        % 
        % 
        %     % End-effector COM
        %     j = N+1;
        %     obj.AC_all{j} = obj.T_all{N} * obj.T_COM{j};
        %     obj.dACidqi_all{j} = zeros(4,4,N);
        %     for k = 1:N
        %         obj.dACidqi_all{j}(:,:,k) = obj.dAdqi_all{k} * obj.T_COM{j};
        %     end
        % 
        %     obj.dAdq_all = KG3.computeTensor(obj.dAdqi_all, obj.T_all);
        %     obj.dACdq_all = KG3.computeTensor(obj.dACidqi_all, obj.AC_all);
        % 
        % 
        % end
   
        % function obj = FKM(obj, q)
        %     % Computes the forward kinematics of the KG3 for a given joint configuration (q)
        %     obj.q = q;
        %     N = length(q);  % 7 joints
        %     L = length(obj.Ts); % 8 links including end-effector
        %     xi = [0;0;0;0;0;1]; % constant local twist for each joint (all joints revolute)
        % 
        %     % Build rotation matrices for each joint (for clarity, optional)
        %     Rs = cell(1,N);
        %     for i = 1:N
        %         Rs{i} = [cos(q(i)) -sin(q(i)) 0 0;
        %                  sin(q(i))  cos(q(i)) 0 0;
        %                  0           0        1 0;
        %                  0           0        0 1];
        %     end
        % 
        %     % For joint links 1 to N
        %     T_base = eye(4);
        %     obj.T_all = cell(1,N+1); % 8 links
        %     for j = 1:N
        %         A = obj.Ts{j} * Rs{j};
        %         T_base = T_base * A;
        %         obj.T_all{j} = T_base;
        % 
        %         % Computing the derivative of each Ai-1_i transformation
        %         obj.dAdqi_all{j} = KG3.hatSE3(xi)*A;   
        %     end
        %     % End-effector
        %     T_base = T_base * obj.Ts{N+1};
        %     obj.T_all{N+1} = T_base;
        %     obj.dAdqi_all{N+1} = obj.dAdqi_all{N};
        % 
        % 
        %     % % Compute full transforms to each joint
        %     % obj.T_all = cell(1,N);
        %     % obj.dAdqi_all = cell(1,N);
        %     % T_base = eye(4);
        %     % for j = 1:N
        %     %     A = obj.Ts{j} * Rs{j};
        %     %     T_base = T_base * A;
        %     %     obj.T_all{j} = T_base;
        %     % 
        %     %     % Computing the derivative of each Ai-1_i transformation
        %     %     obj.dAdqi_all{j} = KG3.hatSE3(xi)*A;   
        %     % end
        % 
        %     % Compute COM transforms in base frame
        %     obj.AC_all = cell(1,L);
        %     obj.dACidqi_all = cell(1,N);
        %     for j = 1:L
        %         if j == 1
        %             % first link: base frame * COM offset * joint rotation
        %             Ajmin1_Cj = obj.T_COM{1} * Rs{1};
        %             obj.AC_all{1} = Ajmin1_Cj;
        %         elseif j <= N
        %             % other joints: use T_all{j-1} to include previous rotations
        %             Ajmin1_Cj = obj.T_COM{j};
        %             obj.AC_all{j} = obj.T_all{j-1} * Ajmin1_Cj;
        %         else
        %             % 8th link (end-effector): just offset from last joint
        %             Ajmin1_Cj = obj.T_COM{j};
        %             obj.AC_all{j} = obj.T_all{N} * Ajmin1_Cj;
        %         end
        % 
        %         % Computing the COM transformation derivatives
        %         obj.dACidqi_all{j} = KG3.hatSE3(xi)*Ajmin1_Cj;
        %     end
        % 
        %     % Computing the full tensors for the transformation derivatives
        %     obj.dAdq_all = KG3.computeTensor(obj.dAdqi_all, obj.T_all);
        %     obj.dACdq_all = KG3.computeTensor(obj.dACidqi_all, obj.AC_all);
        % 
        % end

       
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


        % function obj = FKM(obj, q)
        % 
        %     obj.q = q;
        %     N = length(q);  % 7 joints
        %     L = length(obj.Ts); % 8 links including end-effector
        %     xi = [0;0;0;0;0;1]; % constant local twist for each joint (all joints revolute)
        % 
        %     % Build the rotation matricies for each joint
        %     Rs = cell(1,N);
        %     for i = 1:N
        %         Rs{i} = [cos(q(i)) -sin(q(i)) 0 0;
        %                  sin(q(i))  cos(q(i)) 0 0;
        %                  0           0        1 0;
        %                  0           0        0 1];
        %     end
        % 
        %     % Use the precomputed frame transformations and the rotation
        %     % matricies to compute homogenous transformation matricies for
        %     % each joint
        %     obj.T_all = cell(1,N);
        %     obj.dAdqi_all = cell(1,N);
        %     T_base = eye(4);
        %     for j = 1:N
        %         A = obj.Ts{j} * Rs{j};
        %         T_base = T_base * A;
        %         obj.T_all{j} = T_base;
        % 
        %         % Computing the derivative of each Ai-1_i transformation
        %         obj.dAdqi_all{j} = KG3.hatSE3(xi)*A;   
        %     end
        % 
        %     % Compute the COM homogenous transformation matricies
        %     obj.AC_all = cell(1,L);
        %     obj.dACidqi_all = cell(1,L);
        %     for j = 1:L
        %         % Ajmin1_Cj = obj.T_COM{j} * Rs{j}
        %         if j == 1
        %             Ajmin1_Cj = Rs{1} * obj.T_COM{1};
        %             obj.AC_all{1} = Ajmin1_Cj;
        %         elseif j <= N
        %             Ajmin1_Cj = Rs{j} * obj.T_COM{j};
        %             obj.AC_all{j} = obj.T_all{j-1} * Ajmin1_Cj;
        %         else
        %             Ajmin1_Cj = obj.T_COM{j};
        %             % 8th link (end-effector) – no Rs, just multiply by previous joint transform
        %             obj.AC_all{j} = obj.T_all{N} * Ajmin1_Cj;
        %         end
        % 
        %         % Computing the COM transformation derivatives
        %         obj.dACidqi_all{j} = KG3.hatSE3(xi)*Ajmin1_Cj;
        %     end
        % 
        % end
        % 
        % function obj = JGEO_COM(obj)
        %     N = 7;
        %     L = 8;
        %     obj.J_COM = cell(1,L);
        %     for i = 1:L
        %         Jci = zeros(6,N); % 6x7 Jacobian for link i
        %         pCi = obj.AC_all{i}(1:3,4); % COM position in base frame
        %         for j = 1:min(i,N) % only joints affecting this link
        %             % 
        %             % if j == 1
        %             %     p_jm1 = [0;0;0];                  % base origin
        %             %     z_jm1 = [0;0;1];                   % base axis
        %             % else
        %             %     p_jm1 = obj.T_all{j-1}(1:3,4);    
        %             %     z_jm1 = obj.T_all{j-1}(1:3,1:3)*[0;0;1];
        %             % end
        %             % Jv = cross(z_jm1, pCi - p_jm1);
        %             % Jw = z_jm1;
        %             % Jci(:,j) = [Jv; Jw];
        % 
        %             p_j = obj.T_all{j}(1:3,4);      % joint origin
        %             z_j = obj.T_all{j}(1:3,1:3)*[0;0;1]; % joint axis in base
        %             Jv = cross(z_j, pCi - p_j);     % linear velocity part
        %             Jw = z_j;                        % angular velocity part
        %             Jci(:,j) = [Jv; Jw];
        %         end
        %         obj.J_COM{i} = Jci;
        %     end
        % 
        % 
        % end


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

        % function obj = JGEO(obj)
        %     % Computes the geometric Jacobian of the KG3 for the end-effector
        % 
        %     pe = obj.T_all{end}(1:3,4);          % EE position
        %     N = numel(obj.T_all)-1;              % number of joints
        %     obj.J = zeros(6,N);
        % 
        %     for i = 1:N
        %         R = obj.T_all{i}(1:3,1:3);       % rotation base->joint i
        %         z = R(:,3);                       % joint axis in base frame
        %         p = obj.T_all{i}(1:3,4);          % joint origin in base frame
        % 
        %         Jp = cross(z, pe - p);
        %         Jo = z;
        % 
        %         obj.J(:,i) = [Jp; Jo];
        %     end
        % end



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
            % Compute derivatives of T_all (base -> COM_i) w.r.t. q using finite differences
            %
            % Inputs:
            %   obj     : robot object with obj.FKM(q) returning T_all
            %   q       : Nx1 joint vector
            %   eps_fd  : finite difference step (default 1e-8)
            %
            % Outputs:
            %   dT      : cell{1..L} of 4x4xN, dT{i}(:,:,k) = dT_i/dq_k
            %   dR      : cell{1..L} of 3x3xN, rotation derivatives
            %   dp      : cell{1..L} of 3xN, translation derivatives
            
            if nargin < 3
                eps_fd = 1e-8;
            end
            
            L = numel(obj.T_all);   % number of links
            n = numel(q);
            
            % Preallocate
            dT = cell(L,1);
            dR = cell(L,1);
            dp = cell(L,1);
            
            % Base transform at current q
            T0 = obj.FKM(q).T_all;
            
            for k = 1:n
                % Perturb joint k
                qp = q; qm = q;
                qp(k) = qp(k) + eps_fd;
                qm(k) = qm(k) - eps_fd;
            
                % Compute forward kinematics at perturbed q
                Tp_all = obj.FKM(qp).T_all;
                Tm_all = obj.FKM(qm).T_all;
            
                for i = 1:L
                    % Finite difference
                    obj.dTdq_all{i}(:,:,k) = (Tp_all{i} - Tm_all{i}) / (2*eps_fd);
                end
            end

        end


        function obj = compute_dT_COM_FD(obj, q, eps_fd)
            % Compute derivatives of COM transforms (obj.AC_all) w.r.t. q using finite differences
            %
            % Inputs:
            %   obj     : robot object with obj.FKM(q) updating obj.AC_all
            %   q       : Nx1 joint vector
            %   eps_fd  : finite difference step (default 1e-8)
            %
            % Outputs:
            %   dT      : cell{1..L} of 4x4xN, dT{i}(:,:,k) = d(T_COM_i)/dq_k
            %   dR      : cell{1..L} of 3x3xN, rotation derivatives
            %   dp      : cell{1..L} of 3xN, translation derivatives
            
            if nargin < 3
                eps_fd = 1e-8;
            end
            
            L = numel(obj.AC_all);   % number of COM links
            n = numel(q);
            
            % Preallocate
            dT = cell(L,1);
            dR = cell(L,1);
            dp = cell(L,1);
            
            % Base COM transforms at current q
            obj = obj.FKM(q);              % Updates obj.AC_all
            T0 = obj.AC_all;
            
            for k = 1:n
                % Perturb joint k
                qp = q; qm = q;
                qp(k) = qp(k) + eps_fd;
                qm(k) = qm(k) - eps_fd;
            
                % Compute forward kinematics at perturbed q
                obj = obj.FKM(qp); Tp_all = obj.AC_all;
                obj = obj.FKM(qm); Tm_all = obj.AC_all;
            
                for i = 1:L
                    % Finite difference
                    % dT{i}(:,:,k) = (Tp_all{i} - Tm_all{i}) / (2*eps_fd);
                    obj.dACdq_all{i}(:,:,k) = (Tp_all{i} - Tm_all{i}) / (2*eps_fd);
                    % dR{i}(:,:,k) = dT{i}(1:3,1:3,k);
                    % dp{i}(:,k)    = dT{i}(1:3,4,k);
                end
            end
           
        end



        % function obj = HESSIAN_GEO_COM(obj)
        %     % Computes the geometric hessian for the COM
        % 
        %     % Number of links and joints
        %     L = length(obj.AC_all); 
        %     N = length(obj.T_all);
        %     dAdq_all = obj.dAdq_all;
        %     dACdq_all = obj.dACdq_all;
        % 
        %     % Extracting position derivatives
        %     % L = length(dACdq_all);
        %     drCdq = cell(1,L);
        %     for i=1:L
        %         dA0Cidq = dACdq_all{i};
        %         drCdq{i} = squeeze(dA0Cidq(1:3,4, :));
        %     end
        % 
        %     % Extracting z position derivatives
        %     dzdq = cell(1,L);
        %     dpdq = cell(1,L);
        %     dzdq{1} = zeros(3, 7);
        %     dpdq{1} = zeros(3, 7);
        %     for i=2:L
        %         dA0idq = dAdq_all{i};
        %         dzdq{i} = squeeze(dA0idq(1:3,3, :));
        %         dpdq{i} = squeeze(dA0idq(1:3,4, :));
        % 
        %     end
        % 
        % 
        % 
        %     % Preallocate storage for Hessians
        %     dJ_COM = cell(1, L);   % each cell will hold a 6xNxN tensor
        %     dJ_COM{1} = zeros(6,N,N);
        % 
        %     for i = 1:L
        %         % COM position
        %         rCi = obj.AC_all{i}(1:3,4);
        % 
        %         % Derivative of COM wrt q
        %         % drCidq = squeeze(obj.dACdq_all{i}(1:3,4,:)); % 3xN
        %         drCidq = drCdq{i};
        % 
        %         % Initialise derivative of Jacobian for COM i
        %         dJci = zeros(6, N, N);
        % 
        %         if i > 1
        % 
        %             % Loop over joints j that affect link i
        %             for j = 1:min(i-1,N)
        %                 % Joint origin and axis
        %                 p_j = obj.T_all{j}(1:3,4);
        %                 z_j = obj.T_all{j}(1:3,1:3)*[0;0;1];
        % 
        %                 % Derivatives wrt all qk
        %                 dpjdq = dpdq{j};
        %                 dzjdq = dzdq{j};
        % 
        %                 % For each derivative wrt qk
        %                 for k = 1:7
        %                     dJv = cross(dzjdq(:,k), (rCi - p_j)) ...
        %                         + cross(z_j, (drCidq(:,k) - dpjdq(:,k)));
        % 
        %                     dJw = dzjdq(:,k);
        % 
        %                     dJci(:,j,k) = [dJv; dJw];
        %                 end
        %             end
        % 
        %         end
        % 
        %         % Store Hessian for COM i
        %         dJ_COM{i} = dJci;
        %     end
        % 
        %     obj.dJdq_COM = dJ_COM;
        % end
        
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
        
        %% Computes the coriolis matrix of the KG3
        function obj = CoriolisMatrix(obj, q, dqdt)
            
            % Updating forward kinematics
            obj = obj.FKM(q);
            T_all =obj.T_all;
            
            % Extracting the rotation matricies and their derivatives
            N = length(T_all);
            R_all = cell(1,N);
            dR0dq_all = cell(1,N);
            for i=1:N
                R_all{i} = T_all{i}(1:3,1:3);
                dR0dq_all{i} = obj.dTdq_all{i}(1:3,1:3,:);
            end

            % Extracting the COM Jacobian derivatives
            dJdq_COM = obj.dJdq_COM;
            
            %% Computing the mass matrix derivatives
            dMdq = zeros(7,7,7);
            for k=1:7
                % Compute dMdq(:,:,k)
                for i=1:7
                    % Complete each term for dMdq(:,:,k)

                    dJdq_COMi = dJdq_COM{i}; % Is currently a 6x8x8, but J_COMi is 6x7
                    mi = obj.mArr{i};
                    R0i = R_all{i};
                    dR0idq = dR0dq_all{i};
                    Icii = obj.I{i};
                    J_COMi = obj.J_COM{i};

                    Si = blkdiag(mi*eye(3), R0i*Icii*R0i.');
                    dSi = blkdiag(zeros(3), dR0idq(:,:,k)*Icii*R0i.' + R0i*Icii*dR0idq(:,:,k).');

                    % dMdq(:,:,k) = dMdq(:,:,k) ...
                    %               + (dJdq_COMi(:,:,k).')*[mi*eye(3), zeros(3); zeros(3), R0i*Icii*R0i.']*J_COMi ...
                    %               + (J_COMi.')*[zeros(3), zeros(3); zeros(3), dR0idq(:,:,k)*Icii*R0i.' + R0i*Icii*dR0idq(:,:,k).']*J_COMi ...
                    %               + (J_COMi.')*[zeros(3), zeros(3); zeros(3), R0i*Icii*R0i]*dJdq_COMi(:,:,k);
                    dMdq(:,:,k) = dMdq(:,:,k) ...
                                  + (dJdq_COMi(:,:,k).')*Si*J_COMi ...
                                  + (J_COMi.')*dSi*J_COMi ...
                                  + (J_COMi.')*Si*dJdq_COMi(:,:,k);


                end

            end

            %% Computing the Christoffel symbols
            Gamma = zeros(7,7,7);
            for i=1:7
                for j=1:7
                    for k=1:7
                        Gamma(i,j,k) = 0.5*(dMdq(k,j,i) + dMdq(k,i,j) - dMdq(i,j,k));
                    end
                end
            end

            %% Computing the Coriolis matrix
            obj.C = zeros(7,7);
            for k=1:7
                for j=1:7
                    for i=1:7
                        obj.C(k,j) = obj.C(k,j) + Gamma(i,j,k)*dqdt(i);
                    end
                end
            end


            %% Testing for skew symmetry
            Mdot = zeros(7,7);
            for k=1:7
                Mdot = Mdot + dMdq(:,:,k) * dqdt(k);
            end
            testMat = Mdot - 2*obj.C;
            norm(testMat + testMat.', 'fro')   % should be ~0

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