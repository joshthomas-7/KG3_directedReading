classdef KG3
    % TODO: ADD CLASS DESCRIPTION

    properties
        Ts
        T_all % all transformation matricies for the KG3 T0i (referred to base frame).
        pose  % pose of the KG3.
        J % geometric jacobian.
        mArr %kg, array each stores the mass of each link in the KG3.
        COM_coords % array wich stores the location of the centre of mass (COM) of each link.
        I    % cells which store the inertia tensors of each link referred to the COM.
    end

    methods
        function obj = KG3()
            %KG3 Construct an instance of this class
            % Precomputes the homogenous transformations between each frame
            % of the KG3 
            [obj.Ts, obj.mArr, obj.I] = KG3.preCompute();
        end


        function obj = FKM(obj, q)
            % Computes the forward kinematics of the KG3 for a given joint
            % configuration (q)

            % Creating the generic rotation matrix for each q
            n = length(q);
            Rs = {};
            for i = 1:n
                R = [cos(q(i)), -sin(q(i)), 0, 0;
                     sin(q(i)),  cos(q(i)), 0, 0;
                     0,        0,       1, 0;
                     0,        0,       0, 1];
                Rs{i} = R;
            end

            % Computing the transformation matrices
            obj.T_all = {};
            T07 = eye(4);
            % T_all{1} = eye(4);
            for j = 1:n
                A = obj.Ts{j} * Rs{j};
                T07 = T07*A;
                obj.T_all{j} = T07;
            end

            T08 = T07*obj.Ts{8};
            obj.T_all{8} = T08;
            r08 = T08(1:end-1,end);
            R = T08(1:3,1:3);
            eul = rotm2eul(R);
            obj.pose = [r08;eul.'];
    
        end

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

        %% Computes the mass matrix of the KG3 in its current joint configuration
        function obj = MassMatrix(obj, q)
            % Computing all of the transformation matricies
            obj = FKM(obj,q);
            % Compute the all of the geometric jacobians

        end

    end

    methods (Static)
        function [Ts, mArr, I] = preCompute()
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
            mArr = [1.697 1.377 1.1636 1.1636 0.930 0.678 0.678 0.364];

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
    end
end