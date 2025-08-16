%%% Joshua Thomas
%%% C3376353

function J = KG3_JGEO(T_all)

% Extracting the end effector position vector
T08 = T_all{end};
pe = T08(1:3,end);

% Constructing each column of the geometric jacobian
N = width(T_all)-1;
J = zeros(6, 7);
for i=1:N

    R = T_all{i}(1:3,1:3);
    zi_min_1 = R(:,3);

    pi_min_1 = T_all{i}(1:3,4);

    Jp = cross(zi_min_1, pe - pi_min_1);
    Jo = zi_min_1;
    J(:,i) = [Jp;Jo];

end

end
