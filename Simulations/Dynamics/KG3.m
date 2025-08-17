classdef KG3
    %KG3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Ts
        T_all % all transformation matricies for the KG3 T0i (referred to base frame)
        pose  % pose of the KG3
    end

    methods
        function obj = KG3()
            %KG3 Construct an instance of this class
            % Precomputes the homogenous transformations between each frame
            % of the KG3 
            obj.Ts = KG3.preCompute();
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

    end

    methods (Static)
        function [Ts] = preCompute()
            
            %% Setting up the direct kinematics  
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
        end
    end
end