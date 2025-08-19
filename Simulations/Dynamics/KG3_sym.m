classdef KG3_sym
    properties
        Ts
        T_all
        pose
        J
        mArr
        T_COM
        r_COM
        AC_all
        J_COM
        I
        q     % now symbolic
        M
        C
        M_fun
        dM_fun
    end

    methods
        function obj = KG3_sym()
            % Precompute fixed transforms, masses, COMs, inertias
            [obj.Ts, obj.mArr, obj.T_COM, obj.I, obj.r_COM] = KG3_sym.preCompute();
            
            % Define symbolic joint variables
            syms q1 q2 q3 q4 q5 q6 q7 real
            obj.q = [q1 q2 q3 q4 q5 q6 q7];
        end

        function obj = FKM(obj)
            q = obj.q;  % symbolic vector
            N = length(q);
            L = length(obj.Ts);

            % Build symbolic rotation matrices
            Rs = cell(1,N);
            for i = 1:N
                Rs{i} = [cos(q(i)) -sin(q(i)) 0 0;
                         sin(q(i))  cos(q(i)) 0 0;
                         0           0        1 0;
                         0           0        0 1];
            end

            % Forward kinematics for joints
            obj.T_all = cell(1,N);
            T_base = eye(4);
            for j = 1:N
                A = obj.Ts{j} * Rs{j};
                T_base = T_base * A;
                obj.T_all{j} = T_base;
            end

            % Compute COM transforms
            obj.AC_all = cell(1,L);
            for j = 1:L
                if j == 1
                    obj.AC_all{1} = obj.T_COM{1} * Rs{1};
                elseif j <= N
                    obj.AC_all{j} = obj.T_all{j-1} * obj.T_COM{j} * Rs{j};
                else
                    obj.AC_all{j} = obj.T_all{N} * obj.T_COM{j};
                end
            end
        end

        function obj = JGEO_COM(obj)
            N = length(obj.q);
            L = length(obj.T_COM);
            obj.J_COM = cell(1,L);

            for i = 1:L
                Jci = sym(zeros(6,N)); % symbolic
                pCi = obj.AC_all{i}(1:3,4); 
                for j = 1:min(i,N)
                    p_j = obj.T_all{j}(1:3,4);      
                    z_j = obj.T_all{j}(1:3,1:3)*[0;0;1]; 
                    Jv = cross(z_j, pCi - p_j);    
                    Jw = z_j;                       
                    Jci(:,j) = [Jv; Jw];
                end
                obj.J_COM{i} = Jci;
            end
        end

        function obj = MassMatrix(obj)
            obj = obj.FKM();
            obj = obj.JGEO_COM();
            N = length(obj.q);
            L = length(obj.Ts);
            obj.M = sym(zeros(N,N));
            for i = 1:L
                mi = obj.mArr{i};
                ICi = obj.I{i};
                R0i = obj.AC_all{i}(1:3,1:3);
                Jci = obj.J_COM{i};
                M_link = Jci.' * [mi*eye(3) zeros(3); zeros(3) R0i*ICi*R0i.'] * Jci;
                obj.M = obj.M + M_link;
            end
        end

        function obj = buildSymbolicCoriolisPipeline(obj, varargin)
            % Build symbolic grads of M(q) once and compile to fast numeric functions.
            % Usage (examples):
            %   obj = obj.MassMatrix();                % ensure obj.M is symbolic
            %   obj = obj.buildSymbolicCoriolisPipeline();                 % in-memory
            %   obj = obj.buildSymbolicCoriolisPipeline('writeFiles',true) % generate .m files
            
                p = inputParser;
                addParameter(p,'writeFiles',false,@islogical);  % optionally write .m files
                parse(p,varargin{:});
                writeFiles = p.Results.writeFiles;
            
                % --- prerequisites ---
                assert(isa(obj.q,'sym'), 'q must be symbolic.');
                assert(isa(obj.M,'sym'), 'Call obj.MassMatrix() first to build symbolic M(q).');
            
                N = numel(obj.q);
            
                % --- compute all partial derivatives dM/dq_k ---
                dM = sym(zeros(N,N,N));
                for k = 1:N
                    dM(:,:,k) = diff(obj.M, obj.q(k));
                end
            
                % --- compile numeric functions with matlabFunction ---
                % Mass matrix
                if writeFiles
                    matlabFunction(obj.M, 'Vars',{obj.q}, 'File','KG3_M_fun');
                    obj.M_fun = @KG3_M_fun;
                else
                    obj.M_fun = matlabFunction(obj.M, 'Vars',{obj.q});
                end
            
                % Each slice dM(:,:,k)
                obj.dM_fun = cell(1,N);
                for k = 1:N
                    if writeFiles
                        fname = sprintf('KG3_dM_k%d_fun',k);
                        matlabFunction(dM(:,:,k), 'Vars',{obj.q}, 'File', fname);
                        obj.dM_fun{k} = str2func(fname);
                    else
                        obj.dM_fun{k} = matlabFunction(dM(:,:,k), 'Vars',{obj.q});
                    end
                end
        end


    
        % function obj = CoriolisMatrix(obj)
        %     % Ensure q and dq are symbolic
        %     N = 7;
        %     syms dq [1 N] real
        % 
        %     % Mass matrix (symbolic)
        %     M = obj.M;  % assume obj.M already symbolic
        % 
        %     % Precompute partial derivatives dM_ij/dq_k
        %     dM = sym(zeros(N,N,N));  % 3D array: (i,j,k)
        %     for k = 1:N
        %         dM(:,:,k) = diff(M, obj.q(k));
        %     end
        % 
        %     % Initialize Coriolis matrix
        %     obj.C = sym(zeros(N,N));
        % 
        %     % Vectorized computation of Christoffel symbols
        %     for i = 1:N
        %         for j = 1:N
        %             % sum over k
        %             Cij = 0;
        %             for k = 1:N
        %                 Cij = Cij + 0.5*( dM(i,j,k) + dM(i,k,j) - dM(j,k,i) )*dq(k);
        %             end
        %             obj.C(i,j) = Cij;
        %         end
        %     end
        % end


    
    end

    methods (Static)
        function [Ts, mArr, TC, I, rC] = preCompute()
            [Ts, mArr, TC, I, rC] = KG3.preCompute();
        end
    end
end
