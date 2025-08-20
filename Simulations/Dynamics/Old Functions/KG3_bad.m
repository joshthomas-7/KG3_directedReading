classdef KG3_bad
    % KG3 Class representing the KG3 manipulator with kinematics and dynamics
    
    properties
        %% FKM properties
        Ts          % Precomputed joint transforms
        T_all       % All transformation matrices to each frame
        dAdqi_all   % Derivatives of individual transforms wrt q
        dAdq_all    % Tensors of full derivatives wrt q

        pose        % Pose of the KG3

        T_COM       % COM transforms relative to preceding link frame
        r_COM       % COM position vectors

        AC_all      % COM transforms in base frame
        dACidqi_all % Derivatives of COM transforms wrt q
        dACdq_all   % Tensors of full derivatives wrt q

        %% Differential Kinematics
        J           % Geometric Jacobian
        J_COM       % COM geometric Jacobians
        dJdq_COM    % COM geometric Hessians

        %% Dynamics
        mArr        % Mass of each link
        I           % Inertia tensors
        q           % Current joint configuration
        M           % Mass matrix
    end

    methods
        function obj = KG3()
            % Constructor: Precompute transforms, masses, COMs, inertias
            [obj.Ts, obj.mArr, obj.T_COM, obj.I, obj.r_COM] = KG3.preCompute();
        end

        function obj = FKM(obj, q)
            % Forward kinematics for given joint configuration q
            obj.q = q;
            N = length(q);
            L = length(obj.Ts);

            xi = [0;0;0;0;0;1];  % All revolute joints along z

            % Rotation matrices for joints
            Rs = cell(1,N);
            for i = 1:N
                Rs{i} = [cos(q(i)) -sin(q(i)) 0 0;
                         sin(q(i))  cos(q(i)) 0 0;
                         0           0        1 0;
                         0           0        0 1];
            end

            % Forward kinematics for joints
            T_base = eye(4);
            obj.T_all = cell(1,N+1);
            obj.dAdqi_all = cell(1,N+1);

            for j = 1:N
                A = obj.Ts{j} * Rs{j};
                T_base = T_base * A;
                obj.T_all{j} = T_base;

                obj.dAdqi_all{j} = KG3.hatSE3(xi) * A;
            end

            % End-effector transform
            T_base = T_base * obj.Ts{N+1};
            obj.T_all{N+1} = T_base;

            % Derivatives for end-effector
            obj.dAdqi_all{N+1} = zeros(4,4,N);
            for j = 1:N
                obj.dAdqi_all{N+1}(:,:,j) = obj.dAdqi_all{j} * obj.Ts{N+1};
            end

            % Compute full tensors
            obj.dAdq_all = KG3.computeTensor(obj.dAdqi_all, obj.T_all);

            % COM transforms
            obj.AC_all = cell(1,L);
            obj.dACidqi_all = cell(1,L);
            for j = 1:L
                if j == 1
                    obj.AC_all{1} = obj.T_COM{1};
                else
                    obj.AC_all{j} = obj.T_all{j-1} * obj.T_COM{j};
                end
            end

            % COM derivatives
            for j = 1:L
                Nq = length(q);
                obj.dACidqi_all{j} = zeros(4,4,Nq);
                for k = 1:min(j,Nq)
                    obj.dACidqi_all{j}(:,:,k) = obj.dAdqi_all{k} * obj.T_COM{j};
                end
            end
            obj.dACdq_all = KG3.computeTensor(obj.dACidqi_all, obj.AC_all);
        end

        function obj = JGEO_COM(obj)
            % Geometric Jacobians for COM of each link
            N = length(obj.q);
            L = length(obj.AC_all);
            obj.J_COM = cell(1,L);

            for i = 1:L
                Jci = zeros(6,N);
                pCi = obj.AC_all{i}(1:3,4);

                for j = 1:min(i-1,N)
                    p_j = obj.T_all{j}(1:3,4);
                    z_j = obj.T_all{j}(1:3,1:3) * [0;0;1];
                    Jv = cross(z_j, pCi - p_j);
                    Jw = z_j;
                    Jci(:,j) = [Jv; Jw];
                end

                obj.J_COM{i} = Jci;
            end
        end

        function obj = JGEO(obj)
            % Geometric Jacobian for end-effector
            pe = obj.T_all{end}(1:3,4);
            N = length(obj.q);
            obj.J = zeros(6,N);
            for i = 1:N
                R = obj.T_all{i}(1:3,1:3);
                z = R(:,3);
                p = obj.T_all{i}(1:3,4);
                obj.J(:,i) = [cross(z, pe - p); z];
            end
        end

        function obj = HESSIAN_GEO_COM(obj)
            % Geometric Hessians for COM of each link
            N = length(obj.q);
            L = length(obj.AC_all);

            % Precompute position derivatives
            drCdq = cell(1,L);
            for i = 1:L
                dA = obj.dACdq_all{i};
                drCdq{i} = squeeze(dA(1:3,4,:));
            end

            % Precompute z and p derivatives
            dzdq = cell(1,L);
            dpdq = cell(1,L);
            dzdq{1} = zeros(3,N);
            dpdq{1} = zeros(3,N);
            for i = 2:L
                dA = obj.dAdq_all{i};
                dzdq{i} = squeeze(dA(1:3,3,:));
                dpdq{i} = squeeze(dA(1:3,4,:));
            end

            % Allocate Hessians
            dJ_COM = cell(1,L);

            for i = 1:L
                rCi = obj.AC_all{i}(1:3,4);
                drCidq = drCdq{i};
                dJci = zeros(6,N,N);

                for j = 1:min(i-1,N)
                    p_j = obj.T_all{j}(1:3,4);
                    z_j = obj.T_all{j}(1:3,1:3)*[0;0;1];

                    dpjdq = dpdq{j};
                    dzjdq = dzdq{j};

                    for k = 1:N
                        dJv = cross(dzjdq(:,k), rCi - p_j) + cross(z_j, drCidq(:,k) - dpjdq(:,k));
                        dJw = dzjdq(:,k);
                        dJci(:,j,k) = [dJv; dJw];
                    end
                end

                dJ_COM{i} = dJci;
            end

            obj.dJdq_COM = dJ_COM;
        end

        function obj = MassMatrix(obj, q)
            % Mass matrix in current configuration
            obj = obj.FKM(q);
            obj = obj.JGEO_COM();

            N = length(q);
            L = length(obj.Ts);
            obj.M = zeros(N,N);

            for i = 1:L
                mi = obj.mArr{i};
                ICi = obj.I{i};
                R0i = obj.AC_all{i}(1:3,1:3);
                Jci = obj.J_COM{i};
                M_link = Jci' * [mi*eye(3) zeros(3); zeros(3) R0i*ICi*R0i'] * Jci;
                obj.M = obj.M + M_link;
            end
        end
    end

    methods (Static)
        function [Ts, mArr, TC, I, rC] = preCompute()
            % Precomputed transforms, COMs, masses, inertias

            Ts = cell(1,8);
            Ts{1} = [1 0 0 0; 0 -1 0 0; 0 0 -1 0.1564; 0 0 0 1];
            Ts{2} = [1 0 0 0; 0 0 -1 0.0054; 0 1 0 -0.1284; 0 0 0 1];
            Ts{3} = [1 0 0 0; 0 0 1 -0.2104; 0 -1 0 -0.0064; 0 0 0 1];
            Ts{4} = [1 0 0 0; 0 0 -1 0.0064; 0 1 0 -0.2104; 0 0 0 1];
            Ts{5} = [1 0 0 0; 0 0 1 -0.2084; 0 -1 0 -0.0064; 0 0 0 1];
            Ts{6} = [1 0 0 0; 0 0 -1 0; 0 1 0 -0.1059; 0 0 0 1];
            Ts{7} = [1 0 0 0; 0 0 1 -0.1059; 0 -1 0 0; 0 0 0 1];
            Ts{8} = [1 0 0 0; 0 -1 0 0; 0 0 -1 -0.0615; 0 0 0 1];

            mArr = {1.697,1.377,1.1636,1.1636,0.930,0.678,0.678,0.364};

            rC = {[ -0.000648;-0.000166;0.084487], ...
                  [-0.000023; -0.010364; -0.073360], ...
                  [-0.000044; -0.099580; -0.013278], ...
                  [-0.000044; -0.006641; -0.117892], ...
                  [-0.000018; -0.075478; -0.015006], ...
                  [0.000001; -0.009432; -0.063883], ...
                  [0.000001; -0.045483; -0.009650], ...
                  [-0.000093; 0.000132; -0.022905]};

            TC = cell(1,8);
            for i=1:8
                TC{i} = [eye(3) rC{i}; zeros(1,3) 1];
            end

            I_vecs = [0.004622 0.000009 0.000060 0.004495 0.000009 0.002079;
                      0.004570 0.000001 0.000002 0.004831 0.000448 0.001409;
                      0.011088 0.000005 0.000000 0.001072 -0.000691 0.011255;
                      0.010932 0.000000 -0.000007 0.011127 0.000606 0.001043;
                      0.008147 -0.000001 0.000000 0.000631 -0.000500 0.008316;
                      0.001596 0.000000 0.000000 0.001607 0.000256 0.000399;
                      0.001641 0.000000 0.000000 0.000410 -0.000278 0.001641;
                      0.000214 0.000000 0.000001 0.000223 -0.000002 0.000240];

            I = cell(1,8);
            for i=1:8
                Ii = I_vecs(i,:);
                I{i} = [Ii(1) Ii(2) Ii(3);
                        Ii(2) Ii(4) Ii(5);
                        Ii(3) Ii(5) Ii(6)];
            end
        end

        function G = hatSE3(xi)
            G = [KG3.skew(xi(4:6)) xi(1:3); zeros(1,4)];
        end

        function S = skew(u)
            S = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
        end

        function tensors = computeTensor(dAdqi_all, T_all)
            N = size(dAdqi_all{1},3);
            L = length(T_all);
            tensors = cell(1,L);
            for i = 1:L
                tensors{i} = zeros(4,4,N);
                for j = 1:min(i,N)
                    Aprod = eye(4);
                    for k = j+1:i
                        Aprod = Aprod * T_all{k};
                    end
                    tensors{i}(:,:,j) = dAdqi_all{j}(:,:,j) * Aprod;
                end
            end
        end
    end
end
