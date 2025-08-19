classdef KG3
    % TODO: ADD CLASS DESCRIPTION

    properties
        %% FKM properties
        Ts
        T_all % all transformation matricies for the KG3 T0i (referred to base frame).
        dAdqi_all % cells which store the derivatives of the homogenous transformation matricies
        dAdq_all  % cells which store the tensors for the derivatives of the homogenous transformation matricies 

        pose  % pose of the KG3.

        T_COM % cells which store the transformations between the COMs and the preeceding link frames.
        r_COM

        AC_all
        dACidqi_all % cells which store the derivatives of the COM homogeneous transformation matricies
        dACdq_all   % cells which store the tensors for the derivatives of the COM homogenous transformation matricies 

        %% Differential Kinematics Properties
        J % geometric jacobian.
        J_COM % cells which store the COM geometric Jacobians.
        
        %% Dynamics Properties
        mArr %kg, array each stores the mass of each link in the KG3.
        I    % cells which store the inertia tensors of each link referred to the COM.
        q     % current joint configuration
        M     % mass matrix 
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
        
            % Compute full transforms to each joint
            obj.T_all = cell(1,N);
            obj.dAdqi_all = cell(1,N);
            T_base = eye(4);
            for j = 1:N
                A = obj.Ts{j} * Rs{j};
                T_base = T_base * A;
                obj.T_all{j} = T_base;

                % Computing the derivative of each Ai-1_i transformation
                obj.dAdqi_all{j} = KG3.hatSE3(xi)*A;   
            end
        
            % Compute COM transforms in base frame
            obj.AC_all = cell(1,L);
            obj.dACidqi_all = cell(1,N);
            for j = 1:L
                if j == 1
                    % first link: base frame * COM offset * joint rotation
                    Ajmin1_Cj = obj.T_COM{1} * Rs{1};
                    obj.AC_all{1} = Ajmin1_Cj;
                elseif j <= N
                    % other joints: use T_all{j-1} to include previous rotations
                    Ajmin1_Cj = obj.T_COM{j};
                    obj.AC_all{j} = obj.T_all{j-1} * Ajmin1_Cj;
                else
                    % 8th link (end-effector): just offset from last joint
                    Ajmin1_Cj = obj.T_COM{j};
                    obj.AC_all{j} = obj.T_all{N} * Ajmin1_Cj;
                end

                % Computing the COM transformation derivatives
                obj.dACidqi_all{j} = KG3.hatSE3(xi)*Ajmin1_Cj;
            end

            % Computing the full tensors for the transformation derivatives
            obj.dAdq_all = KG3.computeTensor(obj.dAdqi_all, obj.T_all);
            obj.dACdq_all = KG3.computeTensor(obj.dACidqi_all, obj.AC_all);

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
        % Computes the gemoetric jacobian of the KG3 in its current
        % configuration

        % Extracting the end effector position vector
        T08 = obj.T_all{end};
        pe = T08(1:3,end);
        
        % Constructing each column of the geometric jacobian
        N = width(obj.T_all)-1;
        obj.J = zeros(6, 7);
        for i=1:N
        
            R = obj.T_all{i}(1:3,1:3);
            zi_min_1 = R(:,3);
        
            pi_min_1 = obj.T_all{i}(1:3,4);
        
            Jp = cross(zi_min_1, pe - pi_min_1);
            Jo = zi_min_1;
            obj.J(:,i) = [Jp;Jo];
        
        end
        
        end

        function obj = HESSIAN_GEO_COM(obj)
            % Computes the geometric hessian for the COM
            
            % Up

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
                M_link = Jci' * [mi*eye(3) zeros(3); zeros(3) R0i*ICi*R0i'] * Jci;
                obj.M = obj.M + M_link;
            end

        end

        % function obj = CoriolisMatrix(obj, q, dq)

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
            mArr = {1.697, 1.377, 1.1636, 1.1636, 0.930, 0.678, 0.678, 0.364};

            % Defining the centre of mass position vectors (pCi = pi + rCi)
            rC = {};
            rC{1} = [-0.000648;-0.000166;0.084487];
            rC{2} = [-0.000023; -0.010364; -0.073360];
            rC{3} = [-0.000044; -0.099580; -0.013278];
            rC{4} = [-0.000044; -0.006641; -0.117892];
            rC{5} = [-0.000018; -0.075478; -0.015006];
            rC{6} = [0.000001; -0.009432; -0.063883];
            rC{7} = [0.000001; -0.045483; -0.009650];
            rC{8} = [-0.000093; 0.000132; -0.022905];

            TC = {};

            for i=1:8
                TC{i} = [eye(3,3) rC{i};
                         zeros(1,3) 1];
            end


            
            % Defining the inertia tensors for each link
            % Ii = [Ixx Ixy Ixz Iyy Iyz Izz]
            % I_vecs = [I0;I2;...I7]
            I_vecs = [0.004622 0.000009 0.000060 0.004495 0.000009 0.002079;
                      0.004570 0.000001 0.000002 0.004831 0.000448 0.001409;
                      0.011088 0.000005 0.000000 0.001072 -0.000691 0.011255;
                      0.010932 0.000000 -0.000007 0.011127 0.000606 0.001043;
                      0.008147 -0.000001 0.000000 0.000631 -0.000500 0.008316;
                      0.001596 0.000000 0.000000 0.001607 0.000256 0.000399;
                      0.001641 0.000000 0.000000 0.000410 -0.000278 0.001641;
                      0.000214 0.000000 0.000001 0.000223 -0.000002 0.000240];
            
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

            tensors = {dA01dq,dA02dq,dA03dq,dA04dq,dA05dq,dA06dq,dA07dq};

        end

    end
end